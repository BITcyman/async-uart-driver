use alloc::collections::VecDeque;
use alloc::sync::Arc;
use alloc::boxed::Box;
use core::{convert::Infallible, pin::Pin};
use core::sync::atomic::{AtomicIsize, AtomicUsize, AtomicBool};
use core::sync::atomic::Ordering::Relaxed;
use core::future::Future;
use core::task::{Context, Poll, Waker};



use embedded_hal::serial::{Read, Write};

use qemu_16550_pac::uart as uart;
use heapless::spsc;
use spin::Mutex;

use crate::task::{Executor, Task};
use crate::waker::from_task;

pub const FIFO_DEPTH: usize = 16;
pub const RTS_PULSE_WIDTH: usize = 8;
pub const SERIAL_NUM: usize = 4;
pub const SERIAL_BASE_ADDRESS: usize = 0x6000_1000;
pub const SERIAL_ADDRESS_STRIDE: usize = 0x1000;

pub const DEFAULT_TX_BUFFER_SIZE: usize = 5256;
pub const DEFAULT_RX_BUFFER_SIZE: usize = 5256;

pub struct BufferedSerial {
    base_address: usize,
    pub rx_buffer: VecDeque<u8>,
    pub tx_buffer: VecDeque<u8>,
    pub rx_count: usize,
    pub tx_count: usize,
    pub intr_count: usize,
    pub rx_intr_count: usize,
    pub tx_intr_count: usize,
    pub rx_fifo_count: usize,
    pub tx_fifo_count: isize,
    rx_intr_enabled: bool,
    tx_intr_enabled: bool,
    prev_cts: bool,
}

impl BufferedSerial {
    pub fn new(base_address: usize) -> Self {
        BufferedSerial {
            // hardware: SerialHardware::new(base_address),
            base_address,
            rx_buffer: VecDeque::with_capacity(DEFAULT_RX_BUFFER_SIZE),
            tx_buffer: VecDeque::with_capacity(DEFAULT_TX_BUFFER_SIZE),
            rx_count: 0,
            tx_count: 0,
            intr_count: 0,
            rx_intr_count: 0,
            tx_intr_count: 0,
            rx_fifo_count: 0,
            tx_fifo_count: 0,
            rx_intr_enabled: false,
            tx_intr_enabled: false,
            prev_cts: true,
        }
    }

    fn hardware(&self) -> &uart::RegisterBlock {
        unsafe { &*(self.base_address as *const _) }
    }

    fn set_divisor(&self, clock: usize, baud_rate: usize) {
        let block = self.hardware();
        let divisor = clock / (16 * baud_rate);
        block.lcr().write(|w| w.dlab().set_bit());
        // #[cfg(board = "board_lrv")]
        // {
        //     block
        //         .dll()
        //         .write(|w| unsafe { w.bits((divisor & 0b1111_1111) as u32) });
        //     block
        //         .dlh()
        //         .write(|w| unsafe { w.bits(((divisor >> 8) & 0b1111_1111) as u32) });
        // }
        // #[cfg(board = "board_qemu")]
        // {
            block
                .dll()
                .write(|w| unsafe { w.bits((divisor & 0b1111_1111) as u8) });
            block
                .dlh()
                .write(|w| unsafe { w.bits(((divisor >> 8) & 0b1111_1111) as u8) });
        // }

        block.lcr().write(|w| w.dlab().clear_bit());
    }

    pub(super) fn enable_rdai(&mut self) {
        self.hardware().ier().modify(|_, w| w.erbfi().enable());
        // println!("enable rdai");
        self.rx_intr_enabled = true;
    }

    fn disable_rdai(&mut self) {
        self.hardware().ier().modify(|_, w| w.erbfi().disable());
        // println!("disable rdai");
        self.rx_intr_enabled = false;
    }

    pub(super) fn enable_threi(&mut self) {
        self.hardware().ier().modify(|_, w| w.etbei().enable());
        self.tx_intr_enabled = true;
    }

    fn disable_threi(&mut self) {
        self.hardware().ier().modify(|_, w| w.etbei().disable());
        self.tx_intr_enabled = false;
    }

    fn try_recv(&self) -> Option<u8> {
        let block = self.hardware();
        if block.lsr().read().dr().bit_is_set() {
            Some(block.rbr().read().rbr().bits())
        } else {
            None
        }
    }

    fn send(&self, ch: u8) {
        let block = self.hardware();
        block.thr().write(|w| w.thr().variant(ch));
    }

    pub fn hardware_init(&mut self, baud_rate: usize) {
        let block = self.hardware();
        let _unused = block.msr().read().bits();
        let _unused = block.lsr().read().bits();
        block.lcr().reset();
        // No modem control
        block.mcr().reset();
        block.ier().reset();
        block.fcr().reset();

        // Enable DLAB and Set divisor
        self.set_divisor(100_000_000, baud_rate);
        // Disable DLAB and set word length 8 bits, no parity, 1 stop bit
        block
            .lcr()
            .modify(|_, w| w.dls().eight().pen().disabled().stop().one());
        // Enable FIFO
        block.fcr().write(|w| {
            w.fifoe()
                .set_bit()
                .rfifor()
                .set_bit()
                .xfifor()
                .set_bit()
                .rt()
                .two_less_than_full()
        });
        // Enable loopback
        // block.mcr.modify(|_, w| w.loop_().loop_back());
        // Enable line status & modem status interrupt
        block
            .ier()
            .modify(|_, w| w.elsi().enable().edssi().enable());
        self.rts(true);
        let _unused = self.dcts();

        // Enable received_data_available_interrupt
        self.enable_rdai();
        self.enable_threi();
    }

    #[inline]
    pub fn read_rts(&self) -> bool {
        self.hardware().mcr().read().rts().is_asserted()
    }

    #[inline]
    pub fn rts(&self, is_asserted: bool) {
        self.hardware().mcr().modify(|_, w| w.rts().bit(is_asserted))
    }

    #[inline]
    pub fn cts(&self) -> bool {
        self.hardware().msr().read().cts().bit()
    }

    #[inline]
    pub fn dcts(&self) -> bool {
        self.hardware().msr().read().dcts().bit()
    }

    #[inline]
    fn toggle_threi(&mut self) {
        self.disable_threi();
        self.enable_threi();
    }

    #[inline]
    fn start_tx(&mut self) {
        // assert!(self.tx_fifo_count >= 0);
        // assert!(self.tx_fifo_count <= FIFO_DEPTH as _);
        while self.tx_fifo_count < FIFO_DEPTH as _ {
            if let Some(ch) = self.tx_buffer.pop_front() {
                self.send(ch);
                self.tx_count += 1;
                self.tx_fifo_count += 1;
            } else {
                self.disable_threi();
                break;
            }
        }

        if self.tx_fifo_count == FIFO_DEPTH as _ {
            self.disable_threi();
        }
    }

    // #[cfg(any(board = "board_qemu", board = "board_lrv"))]
    pub fn interrupt_handler(&mut self) {

        use uart::iir::Iid;

        while let Some(int_type) = self.hardware().iir().read().iid().variant() {
            if int_type == Iid::NoInterruptPending {
                break;
            }
            // let intr_id: usize = int_type as u8 as _;
            self.intr_count += 1;
            match int_type {
                Iid::ReceivedDataAvailable | Iid::CharacterTimeout => {
                    // println!("[SERIAL] Received data available");
                    self.rx_intr_count += 1;
                    while let Some(ch) = self.try_recv() {
                        self.rx_count += 1;
                        self.rx_fifo_count += 1;
                        if self.rx_fifo_count == RTS_PULSE_WIDTH {
                            self.rts(false);
                        } else if self.rx_fifo_count == RTS_PULSE_WIDTH * 2 {
                            self.rts(true);
                            self.rx_fifo_count = 0;
                        }
                        self.rx_buffer.push_back(ch);
                        if self.rx_buffer.len() >= DEFAULT_TX_BUFFER_SIZE {
                            // println!("[USER UART] Serial rx buffer overflow!");
                            self.disable_rdai();
                            break;
                        }
                    }
                }
                Iid::ThrEmpty => {
                    self.tx_intr_count += 1;
                    // println!("[SERIAL] Transmitter Holding Register Empty");
                    self.start_tx();
                }
                Iid::ReceiverLineStatus => {
                    let block = self.hardware();
                    let lsr = block.lsr().read();
                    // if lsr.bi().bit_is_set() {
                    if lsr.fifoerr().is_error() {
                        if lsr.bi().bit_is_set() {
                            // println!("[uart] lsr.BI!");
                        }
                        if lsr.fe().bit_is_set() {
                            // println!("[uart] lsr.FE!");
                        }
                        if lsr.pe().bit_is_set() {
                            // println!("[uart] lsr.PE!");
                        }
                    }
                    if lsr.oe().bit_is_set() {
                        block.mcr().modify(|_, w| w.rts().deasserted());
                        // println!("[uart] lsr.OE!");
                    }
                }
                Iid::ModemStatus => {
                    if self.dcts() {
                        let cts = self.cts();
                        if cts == self.prev_cts {
                            // while !self.hardware().lsr.read().thre().is_empty() {}
                            self.tx_fifo_count -= (RTS_PULSE_WIDTH * 2) as isize;
                        } else {
                            self.tx_fifo_count -= RTS_PULSE_WIDTH as isize;
                        }
                        self.prev_cts = cts;
                        self.toggle_threi();
                        self.start_tx();
                    } else {
                        let _block = self.hardware();
                        // println!(
                        //     "[USER SERIAL] EDSSI, MSR: {:#x}, LSR: {:#x}, IER: {:#x}",
                        //     block.msr.read().bits(),
                        //     block.lsr.read().bits(),
                        //     block.ier().read().bits()
                        // );
                    }
                }
                _ => {
                    // warn!("[USER SERIAL] {:?} not supported!", int_type);
                }
            }
        }
    }
}

impl Write<u8> for BufferedSerial {
    type Error = Infallible;

    // #[cfg(any(feature = "board_qemu", feature = "board_lrv"))]
    fn try_write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        if self.tx_buffer.len() < DEFAULT_TX_BUFFER_SIZE {
            self.tx_buffer.push_back(word);
            if self.tx_fifo_count < FIFO_DEPTH as _ {
                self.toggle_threi();
                self.start_tx();
            }
        } else {
            // println!("[USER SERIAL] Tx buffer overflow!");
            return Err(nb::Error::WouldBlock);
        }
        Ok(())
    }

    fn try_flush(&mut self) -> nb::Result<(), Self::Error> {
        todo!()
    }
}

impl Read<u8> for BufferedSerial {
    type Error = Infallible;

    fn try_read(&mut self) -> nb::Result<u8, Self::Error> {
        if let Some(ch) = self.rx_buffer.pop_front() {
            Ok(ch)
        } else {
            if !self.rx_intr_enabled {
                self.enable_rdai();
            }
            Err(nb::Error::WouldBlock)
        }
    }
}

impl Drop for BufferedSerial {
    fn drop(&mut self) {
        let block = self.hardware();
        block.ier().reset();
        let _unused = block.msr().read().bits();
        let _unused = block.lsr().read().bits();
        self.rts(false);
        // reset Rx & Tx FIFO, disable FIFO
        block
            .fcr()
            .write(|w| w.fifoe().clear_bit().rfifor().set_bit().xfifor().set_bit());
    }
}


type RxProducer = spsc::Producer<'static, u8, DEFAULT_RX_BUFFER_SIZE>;
type RxConsumer = spsc::Consumer<'static, u8, DEFAULT_RX_BUFFER_SIZE>;
type TxProducer = spsc::Producer<'static, u8, DEFAULT_TX_BUFFER_SIZE>;
type TxConsumer = spsc::Consumer<'static, u8, DEFAULT_TX_BUFFER_SIZE>;

pub struct AsyncSerial {
    base_address: usize,
    rx_pro: Mutex<RxProducer>,
    rx_con: Mutex<RxConsumer>,
    tx_pro: Mutex<TxProducer>,
    tx_con: Mutex<TxConsumer>,
    pub rx_count: AtomicUsize,
    pub tx_count: AtomicUsize,
    pub intr_count: AtomicUsize,
    pub rx_intr_count: AtomicUsize,
    pub tx_intr_count: AtomicUsize,
    rx_fifo_count: AtomicUsize,
    tx_fifo_count: AtomicIsize,
    pub(super) rx_intr_enabled: AtomicBool,
    pub(super) tx_intr_enabled: AtomicBool,
    prev_cts: AtomicBool,
    read_wakers: Mutex<VecDeque<Waker>>,
    write_wakers: Mutex<VecDeque<Waker>>,
    executor: Executor,
}

impl AsyncSerial {
    pub fn new(
        base_address: usize,
        rx_pro: RxProducer,
        rx_con: RxConsumer,
        tx_pro: TxProducer,
        tx_con: TxConsumer,
    ) -> Self {
        AsyncSerial {
            base_address,
            rx_pro: Mutex::new(rx_pro),
            rx_con: Mutex::new(rx_con),
            tx_pro: Mutex::new(tx_pro),
            tx_con: Mutex::new(tx_con),
            rx_count: AtomicUsize::new(0),
            tx_count: AtomicUsize::new(0),
            intr_count: AtomicUsize::new(0),
            rx_intr_count: AtomicUsize::new(0),
            tx_intr_count: AtomicUsize::new(0),
            rx_fifo_count: AtomicUsize::new(0),
            tx_fifo_count: AtomicIsize::new(0),
            rx_intr_enabled: AtomicBool::new(false),
            tx_intr_enabled: AtomicBool::new(false),
            prev_cts: AtomicBool::new(true),
            read_wakers: Mutex::new(VecDeque::new()),
            write_wakers: Mutex::new(VecDeque::new()),
            executor: Executor::default(),
        }
    }

    fn hardware(&self) -> &uart::RegisterBlock {
        unsafe { &*(self.base_address as *const _) }
    }

    fn set_divisor(&self, clock: usize, baud_rate: usize) {
        let block = self.hardware();
        let divisor = clock / (16 * baud_rate);
        block.lcr().write(|w| w.dlab().set_bit());
        // #[cfg(feature = "board_lrv")]
        // {
        //     block
        //         .dll()
        //         .write(|w| unsafe { w.bits((divisor & 0b1111_1111) as u32) });
        //     block
        //         .dlh()
        //         .write(|w| unsafe { w.bits(((divisor >> 8) & 0b1111_1111) as u32) });
        // }
        // #[cfg(feature = "board_qemu")]
        // {
            block
                .dll()
                .write(|w| unsafe { w.bits((divisor & 0b1111_1111) as u8) });
            block
                .dlh()
                .write(|w| unsafe { w.bits(((divisor >> 8) & 0b1111_1111) as u8) });
        // }

        block.lcr().write(|w| w.dlab().clear_bit());
    }

    #[inline]
    fn addr_no(&self) -> usize {
        ((self.base_address >> 12) & 0xFF) + 3
    }

    pub(super) fn enable_rdai(&self) {
        self.hardware().ier().modify(|_, w| w.erbfi().set_bit());
        self.rx_intr_enabled.store(true, Relaxed);
    }

    fn disable_rdai(&self) {
        self.hardware().ier().modify(|_, w| w.erbfi().clear_bit());
        self.rx_intr_enabled.store(false, Relaxed);
    }

    pub(super) fn enable_threi(&self) {
        self.hardware().ier().modify(|_, w| w.etbei().set_bit());
        self.tx_intr_enabled.store(true, Relaxed);
    }

    fn disable_threi(&self) {
        self.hardware().ier().modify(|_, w| w.etbei().clear_bit());
        self.tx_intr_enabled.store(false, Relaxed);
    }

    #[inline]
    pub fn rts(&self, is_asserted: bool) {
        // println!("[uart] rts: {}", is_asserted);
        self.hardware().mcr().modify(|_, w| w.rts().bit(is_asserted))
    }

    #[inline]
    pub fn cts(&self) -> bool {
        self.hardware().msr().read().cts().bit()
    }

    #[inline]
    pub fn dcts(&self) -> bool {
        self.hardware().msr().read().dcts().bit()
    }

    fn try_recv(&self) -> Option<u8> {
        let block = self.hardware();
        if block.lsr().read().dr().bit_is_set() {
            let ch = block.rbr().read().rbr().bits();
            // push_trace(SERIAL_RX | ch as usize);
            Some(ch)
        } else {
            None
        }
    }

    fn send(&self, ch: u8) {
        let block = self.hardware();
        // push_trace(SERIAL_TX | ch as usize);
        block.thr().write(|w| w.thr().variant(ch));
    }

    pub(super) fn try_read(&self) -> Option<u8> {
        if let Some(mut rx_lock) = self.rx_con.try_lock() {
            rx_lock.dequeue()
        } else {
            // println!("[async] cannot lock rx queue!");
            None
        }
    }

    pub(super) fn try_write(&self, ch: u8) -> Result<(), u8> {
        if let Some(mut tx_lock) = self.tx_pro.try_lock() {
            // log::debug!("tx_len is {}",tx_lock.len());
            tx_lock.enqueue(ch)
        } else {
            // println!("[async] cannot lock tx queue!");
            Err(ch)
        }
    }

    pub fn hardware_init(&self, baud_rate: usize) {
        let block = self.hardware();
        let _unused = block.msr().read().bits();
        let _unused = block.lsr().read().bits();
        block.lcr().reset();
        // No modem control
        block.mcr().reset();
        block.ier().reset();
        block.fcr().reset();

        // Enable DLAB and Set divisor
        self.set_divisor(100_000_000, baud_rate);
        // Disable DLAB and set word length 8 bits, no parity, 1 stop bit
        block
            .lcr()
            .modify(|_, w| w.dls().eight().pen().disabled().stop().one());
        // Enable FIFO
        block.fcr().write(|w| {
            w.fifoe()
                .set_bit()
                .rfifor()
                .set_bit()
                .xfifor()
                .set_bit()
                .rt()
                .two_less_than_full()
        });
        self.rts(true);
        let _unused = self.dcts();
        // Enable line status & modem status interrupt
        block
            .ier()
            .modify(|_, w| w.elsi().enable().edssi().enable());
        // Enable received_data_available_interrupt
        self.enable_rdai();
        self.enable_threi();
    }

    #[inline]
    fn toggle_threi(&self) {
        self.disable_threi();
        self.enable_threi();
    }

    #[inline]
    fn start_tx(&self) {
        let mut tx_count = 0;
        let mut tx_fifo_count = self.tx_fifo_count.load(Relaxed);
        // assert!(tx_fifo_count >= 0);
        assert!(tx_fifo_count <= FIFO_DEPTH as _);
        let mut con = self.tx_con.lock();

        while tx_fifo_count < FIFO_DEPTH as _ {
            if let Some(ch) = con.dequeue() {
                log::debug!("tx_con => serial {}", ch);
                self.send(ch);
                tx_count += 1;
                tx_fifo_count += 1;
            } else {
                self.disable_threi();
                break;
            }
        }

        if tx_fifo_count == FIFO_DEPTH as _ {
            self.disable_threi();
        }

        self.tx_count.fetch_add(tx_count, Relaxed);
        self.tx_fifo_count.store(tx_fifo_count, Relaxed);
    }

    pub fn interrupt_handler(&self) {
        log::debug!("[Async Serial] Interrupt!");

        // use crate::trace::{ASYNC_READ_WAKE, ASYNC_WRITE_WAKE};
        use core::sync::atomic::Ordering::{Acquire, Release};
        use uart::iir::Iid;

        let block = self.hardware();
        while let Some(int_type) = block.iir().read().iid().variant() {
            if int_type == Iid::NoInterruptPending {
                break;
            }
            // let intr_id: usize = int_type as u8 as _;
            // push_trace(SERIAL_INTR_ENTER + intr_id);
            self.intr_count.fetch_add(1, Relaxed);
            match int_type {
                Iid::ReceivedDataAvailable | Iid::CharacterTimeout => {
                    log::debug!("[SERIAL] Received data available");
                    self.rx_intr_count.fetch_add(1, Relaxed);
                    let mut rx_count = 0;
                    let mut rx_fifo_count = self.rx_fifo_count.load(Acquire);
                    let mut pro = self.rx_pro.lock();
                    while let Some(ch) = self.try_recv() {
                        rx_fifo_count += 1;
                        rx_count += 1;
                        if rx_fifo_count == RTS_PULSE_WIDTH {
                            // push_trace(SERIAL_RTS);
                            self.rts(false);
                        } else if rx_fifo_count == RTS_PULSE_WIDTH * 2 {
                            // push_trace(SERIAL_RTS | 1);
                            self.rts(true);
                            rx_fifo_count = 0;
                        }
                        if let Err(_) = pro.enqueue(ch) {
                            // println!("[USER UART] Serial rx buffer overflow!");
                        }
                        if pro.len() >= DEFAULT_RX_BUFFER_SIZE - 1 {
                            self.disable_rdai();
                            break;
                        }
                    }
                    self.rx_fifo_count.store(rx_fifo_count, Release);
                    self.rx_count.fetch_add(rx_count, Relaxed);
                    
                    log::debug!("before wake");
                    while let Some(waker) = self.read_wakers.lock().pop_front(){
                        log::debug!("wake read task");
                        waker.wake()
                    };
                    log::debug!("after wake");
                    log::debug!("executor::run_until_idle");
                    self.executor.run_until_idle();
                    
                }
                Iid::ThrEmpty => {
                    log::debug!("[SERIAL] Transmitter Holding Register Empty");
                    self.tx_intr_count.fetch_add(1, Relaxed);
                    self.start_tx();
                }
                Iid::ReceiverLineStatus => {
                    log::debug!("[SERIAL] ReceiverLineStatus");
                    let block = self.hardware();
                    let lsr = block.lsr().read();
                    // if lsr.bi().bit_is_set() {
                    if lsr.fifoerr().is_error() {
                        if lsr.bi().bit_is_set() {
                            // println!("[uart] lsr.BI!");
                        }
                        if lsr.fe().bit_is_set() {
                            // println!("[uart] lsr.FE!");
                        }
                        if lsr.pe().bit_is_set() {
                            // println!("[uart] lsr.PE!");
                        }
                    }
                    if lsr.oe().bit_is_set() {
                        block.mcr().modify(|_, w| w.rts().deasserted());
                        // println!("[uart] lsr.OE!");
                    }
                }
                Iid::ModemStatus => {
                    log::debug!("[SERIAL] ModemStatus");
                    if self.dcts() {
                        let cts = self.cts();
                        if cts == self.prev_cts.load(Relaxed) {
                            // push_trace(SERIAL_CTS | (RTS_PULSE_WIDTH * 2));
                            self.tx_fifo_count
                                .fetch_add(-(RTS_PULSE_WIDTH as isize * 2), Relaxed);
                        } else {
                            // push_trace(SERIAL_CTS | RTS_PULSE_WIDTH);
                            self.tx_fifo_count
                                .fetch_add(-(RTS_PULSE_WIDTH as isize), Relaxed);
                        }
                        self.prev_cts.store(cts, Relaxed);
                        self.toggle_threi();
                        // println!("dcts && cts");
                        while let Some(waker) = self.write_wakers.lock().pop_front() {
                            waker.wake()
                        };

                        self.executor.run_until_idle();

                    } else {
                        // let block = self.hardware();
                        // println!(
                        //     "[USER SERIAL] EDSSI, MSR: {:#x}, LSR: {:#x}, IER: {:#x}",
                        //     block.msr.read().bits(),
                        //     block.lsr.read().bits(),
                        //     block.ier().read().bits()
                        // );
                    }
                }
                _ => {
                    log::debug!("[SERIAL] not supported!");
                }
            }
            // push_trace(SERIAL_INTR_EXIT + intr_id);
        }
    }

    pub async fn read(self: Arc<Self>, buf: &'static mut [u8]) {
        let future = SerialReadFuture {
            buf,
            read_len: 0,
            driver: self.clone(),
        };
        // 注册
        let task = Task::new(Box::pin(future), self.clone(), crate::task::TaskIOType::Read);
        self.register_readwaker( unsafe { from_task(task.clone())} );
        self.executor.push_task(Task::from_ref(task));
    }

    pub async fn write(self: Arc<Self>, buf: &'static [u8]) {
        let future = SerialWriteFuture {
            buf,
            write_len: 0,
            driver: self.clone(),
        };
        let task = Task::new(Box::pin(future), self.clone(), crate::task::TaskIOType::Write);
        self.register_writewaker(
            unsafe { from_task(task.clone()) }
        );
        self.executor.push_task(Task::from_ref(task))
    }

    pub fn register_readwaker(&self, read_waker: Waker) {
        self.read_wakers.lock().push_back(read_waker)
    }

    pub fn register_writewaker(&self, write_waker: Waker) {
        self.write_wakers.lock().push_back(write_waker)
    }

    /// remove all read wakers
    pub fn remove_read(&self) {
        self.read_wakers.lock().clear();
    }

    /// remove all write wakers
    pub fn remove_write(&self) {
        self.write_wakers.lock().clear();
    }
}

impl Drop for AsyncSerial {
    fn drop(&mut self) {
        let block = self.hardware();
        block.ier().reset();
        let _unused = block.msr().read().bits();
        let _unused = block.lsr().read().bits();
        self.rts(false);
        // reset Rx & Tx FIFO, disable FIFO
        block
            .fcr()
            .write(|w| w.fifoe().clear_bit().rfifor().set_bit().xfifor().set_bit());
        // println!("Async driver dropped!");
    }
}

struct SerialReadFuture<'a> {
    buf: &'a mut [u8],
    read_len: usize,
    driver: Arc<AsyncSerial>,
}

unsafe impl Send for SerialReadFuture<'_> {}
unsafe impl Sync for SerialReadFuture<'_> {}




impl Future for SerialReadFuture<'_> {
    type Output = i32;

    fn poll(mut self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<Self::Output> {
        // println!("read poll");
        // let driver = self.driver.clone();
    
        while let Some(data) = self.driver.try_read() {
            if self.read_len < self.buf.len() {
                let len = self.read_len;
                self.buf[len] = data;
                self.read_len += 1;
                log::debug!("read task receive '{:?}'", self.buf);
            } else {
                // println!("### [{:x}] r poll fin ####", self.driver.addr_no());
                // push_trace(ASYNC_READ_POLL);
                return Poll::Ready(self.read_len as i32);
            }
        }

        if !self.driver.rx_intr_enabled.load(Relaxed) {
            // println!("read intr enabled");
            self.driver.enable_rdai();
        }
        // println!("$$$ [{:x}] r poll pen $$$$", driver.addr_no());
        // push_trace(ASYNC_READ_POLL | self.read_len);
        Poll::Pending
    }
}

struct SerialWriteFuture<'a> {
    buf: &'a [u8],
    write_len: usize,
    driver: Arc<AsyncSerial>,
}

unsafe impl Send for SerialWriteFuture<'_> {}
unsafe impl Sync for SerialWriteFuture<'_> {}

impl Future for SerialWriteFuture<'_> {
    type Output = i32;

    fn poll(mut self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<Self::Output> {
        // println!("write poll");
        // let driver = self.driver.clone();

        if self.driver.tx_fifo_count.load(Relaxed) < FIFO_DEPTH as _ {
            // println!("=== [{:x}] w intr en ====", self.driver.addr_no());
            self.driver.toggle_threi();
            self.driver.start_tx();
        }
        while let Ok(()) = self.driver.try_write(self.buf[self.write_len]) {
            if self.write_len < self.buf.len() - 1 {
                self.write_len += 1;
            } else {
                // println!("--- [{:x}] w poll fin ----", self.driver.addr_no());
                // push_trace(ASYNC_WRITE_POLL);
                self.driver.toggle_threi();
                return Poll::Ready(self.write_len as i32);
            }
        }

        // println!("^^^ [{:x}] w poll pen ^^^^", self.driver.addr_no());
        // push_trace(ASYNC_WRITE_POLL | self.write_len);
        Poll::Pending
    }
}