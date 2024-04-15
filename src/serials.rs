use alloc::collections::VecDeque;
use alloc::sync::Arc;
use core::{convert::Infallible, pin::Pin};
use core::sync::atomic::{AtomicIsize, AtomicUsize, AtomicBool};
use core::sync::atomic::Ordering::Relaxed;
use core::future::Future;
use core::task::{Context, Poll, Waker};

use embedded_hal::serial::{Read, Write};

use qemu_16550_pac::uart as uart;
use heapless::spsc;
use alloc::boxed::Box;
use spin::Mutex;
use crate::future::GetWakerFuture;

use ats_intc::*;
/// The basic address of the ATS-INTC
const ATSINTC_BASEADDR: usize = 0x1000_0000;
/// The ATS-INTC driver of kernel
static ATSINTC: AtsIntc = AtsIntc::new(ATSINTC_BASEADDR);
// /// The ATS-INTC driver of process1
// static ATSINTC1: AtsIntc = AtsIntc::new(ATSINTC_BASEADDR + 1 * 0x1000);

pub const FIFO_DEPTH: usize = 16;
pub const RTS_PULSE_WIDTH: usize = 8;
pub const SERIAL_NUM: usize = 4;
pub const SERIAL_BASE_ADDRESS: usize = 0x6000_1000;
pub const SERIAL_ADDRESS_STRIDE: usize = 0x1000;

// pub fn irq_to_serial_id(irq: u16) -> usize {
//     match irq {
//         12 => 0,
//         13 => 1,
//         14 => 2,
//         15 => 3,
//         _ => 0,
//     }
// }

// pub fn get_base_addr_from_irq(irq: u16) -> usize {
//     SERIAL_BASE_ADDRESS + irq_to_serial_id(irq) * SERIAL_ADDRESS_STRIDE
// }

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
                        let block = self.hardware();
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
    serial_rx: Arc<AsyncSerialRx>,
    serial_tx: Arc<AsyncSerialTx>,
    read_waker: Mutex<Option<Waker>>,
    write_waker: Mutex<Option<Waker>>,
}

struct AsyncSerialRx {
    base_address: usize,
    rx_pro: Mutex<RxProducer>,
    rx_con: Mutex<RxConsumer>,
    pub rx_count: AtomicUsize,
    pub rx_intr_count: AtomicUsize,
    rx_fifo_count: AtomicUsize,
    pub(super) rx_intr_enabled: AtomicBool,
    prev_cts: AtomicBool
}

struct AsyncSerialTx {
    base_address: usize,
    tx_pro: Mutex<TxProducer>,
    tx_con: Mutex<TxConsumer>,
    pub tx_count: AtomicUsize,
    pub tx_intr_count: AtomicUsize,
    tx_fifo_count: AtomicIsize,
    pub(super) tx_intr_enabled: AtomicBool,
    prev_cts: AtomicBool
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
            serial_rx: Arc::new(AsyncSerialRx::new(
                base_address,
                rx_pro,
                rx_con
            )),
            serial_tx: Arc::new(AsyncSerialTx::new(
                base_address,
                tx_pro,
                tx_con
            )),
            read_waker: Mutex::new(None),
            write_waker: Mutex::new(None),
        }
    }

    fn hardware(&self) -> &uart::RegisterBlock {
        unsafe { &*(self.base_address as *const _) }
    }

    fn set_divisor(&self, clock: usize, baud_rate: usize) {
        let block = self.hardware();
        let divisor = clock / (16 * baud_rate);
        block.lcr().write(|w| w.dlab().set_bit());
        block
            .dll()
            .write(|w| unsafe { w.bits((divisor & 0b1111_1111) as u8) });
        block
            .dlh()
            .write(|w| unsafe { w.bits(((divisor >> 8) & 0b1111_1111) as u8) });
        block.lcr().write(|w| w.dlab().clear_bit());
    }

    #[inline]
    fn addr_no(&self) -> usize {
        ((self.base_address >> 12) & 0xFF) + 3
    }

    pub(super) fn enable_rdai(&self) {
        self.serial_rx.enable_rdai()
    }

    fn disable_rdai(&self) {
        self.serial_rx.disable_rdai()
    }

    pub(super) fn enable_threi(&self) {
        self.serial_tx.enable_threi()
    }

    fn disable_threi(&self) {
        self.serial_tx.disable_threi()
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
        self.serial_rx.try_recv()
    }

    fn send(&self, ch: u8) {
        self.serial_tx.send(ch)
    }

    pub(super) fn try_read(&self) -> Option<u8> {
        self.serial_rx.try_read()
    }

    pub(super) fn try_write(&self, ch: u8) -> Result<(), u8> {
        self.serial_tx.try_write(ch)
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

    pub fn init_interrupt_handler(self, irq: u16) {
        let handler = Task::new(
            Box::pin(Handler::new(Arc::new(self),irq)),
            1,
            TaskType::AsyncSyscall,
            &ATSINTC
        );
        ATSINTC.intr_push(irq as usize, handler);
    }


    #[inline]
    fn toggle_threi(&self) {
        self.disable_threi();
        self.enable_threi();
    }

    pub async fn read(self: Arc<Self>, buf: &mut [u8]) {
        let future = SerialReadFuture {
            buf,
            read_len: 0,
            driver: self.clone(),
        };
        future.await;
    }

    pub async fn write(self: Arc<Self>, buf: &[u8]) {
        let future = SerialWriteFuture {
            buf,
            write_len: 0,
            driver: self.clone(),
        };
        future.await;
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


impl AsyncSerialTx {
    pub fn new(
        base_address: usize,
        tx_pro: TxProducer,
        tx_con: TxConsumer,
    ) -> Self {
        AsyncSerialTx {
            base_address,
            tx_pro: Mutex::new(tx_pro),
            tx_con: Mutex::new(tx_con),
            tx_count: AtomicUsize::new(0),
            tx_intr_count: AtomicUsize::new(0),
            tx_fifo_count: AtomicIsize::new(0),
            tx_intr_enabled: AtomicBool::new(false),
            prev_cts: AtomicBool::new(true)
        }
    }

    fn hardware(&self) -> &uart::RegisterBlock {
        unsafe { &*(self.base_address as *const _) }
    }

    pub fn enable_threi(&self) {
        self.hardware().ier().modify(|_, w| w.etbei().set_bit());
        self.tx_intr_enabled.store(true, Relaxed);
    }

    fn disable_threi(&self) {
        self.hardware().ier().modify(|_, w| w.etbei().clear_bit());
        self.tx_intr_enabled.store(false, Relaxed);
    }

    /// 初始化 tx 
    #[inline]
    fn start_tx(&self) {
        let mut tx_count = 0;
        let mut tx_fifo_count = self.tx_fifo_count.load(Relaxed);
        // assert!(tx_fifo_count >= 0);
        assert!(tx_fifo_count <= FIFO_DEPTH as _);
        let mut con = self.tx_con.lock();

        while tx_fifo_count < FIFO_DEPTH as _ {
            if let Some(ch) = con.dequeue() {
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

    fn send(&self, ch: u8) {
        let block = self.hardware();
        // push_trace(SERIAL_TX | ch as usize);
        block.thr().write(|w| w.thr().variant(ch));
    }

    pub fn try_write(&self, ch: u8) -> Result<(), u8> {
        if let Some(mut tx_lock) = self.tx_pro.try_lock() {
            tx_lock.enqueue(ch)
        } else {
            // println!("[async] cannot lock tx queue!");
            Err(ch)
        }
    }

}

impl AsyncSerialRx {
    pub fn new(
        base_address: usize,
        rx_pro: RxProducer,
        rx_con: RxConsumer,
    ) -> Self {
        AsyncSerialRx {
            base_address,
            rx_pro: Mutex::new(rx_pro),
            rx_con: Mutex::new(rx_con),
            rx_count: AtomicUsize::new(0),
            rx_intr_count: AtomicUsize::new(0),
            rx_fifo_count: AtomicUsize::new(0),
            rx_intr_enabled: AtomicBool::new(false),
            prev_cts: AtomicBool::new(true)
        }
    }


    fn hardware(&self) -> &uart::RegisterBlock {
        unsafe { &*(self.base_address as *const _) }
    }

    pub fn enable_rdai(&self) {
        self.hardware().ier().modify(|_, w| w.erbfi().set_bit());
        self.rx_intr_enabled.store(true, Relaxed);
    }

    fn disable_rdai(&self) {
        self.hardware().ier().modify(|_, w| w.erbfi().clear_bit());
        self.rx_intr_enabled.store(false, Relaxed);
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

    pub fn try_read(&self) -> Option<u8> {
        if let Some(mut rx_lock) = self.rx_con.try_lock() {
            rx_lock.dequeue()
        } else {
            // println!("[async] cannot lock rx queue!");
            None
        }
    }
}

struct SerialReadFuture<'a> {
    buf: &'a mut [u8],
    read_len: usize,
    driver: Arc<AsyncSerial>,
}

impl Future for SerialReadFuture<'_> {
    type Output = i32;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // println!("read poll");
        // let driver = self.driver.clone();
        while let Some(data) = self.driver.try_read() {
            if self.read_len < self.buf.len() {
                let len = self.read_len;
                self.buf[len] = data;
                self.read_len += 1;
            } else {
                // println!("### [{:x}] r poll fin ####", self.driver.addr_no());
                // push_trace(ASYNC_READ_POLL);
                return Poll::Ready(0);
            }
        }

        if !self.driver.serial_rx.rx_intr_enabled.load(Relaxed) {
            // println!("read intr enabled");
            self.driver.enable_rdai();
        }
        // println!("$$$ [{:x}] r poll pen $$$$", driver.addr_no());
        // push_trace(ASYNC_READ_POLL | self.read_len);
        let waker = cx.waker().clone();


        Poll::Pending
    }
}


struct SerialWriteFuture<'a> {
    buf: &'a [u8],
    write_len: usize,
    driver: Arc<AsyncSerial>,
}

impl Future for SerialWriteFuture<'_> {
    type Output = i32;

    fn poll(mut self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<Self::Output> {
        // println!("write poll");
        // let driver = self.driver.clone();

        if self.driver.serial_tx.tx_fifo_count.load(Relaxed) < FIFO_DEPTH as _ {
            // println!("=== [{:x}] w intr en ====", self.driver.addr_no());
            self.driver.toggle_threi();
            self.driver.serial_tx.start_tx();
        }
        while let Ok(()) = self.driver.try_write(self.buf[self.write_len]) {
            if self.write_len < self.buf.len() - 1 {
                self.write_len += 1;
            } else {
                // println!("--- [{:x}] w poll fin ----", self.driver.addr_no());
                // push_trace(ASYNC_WRITE_POLL);
                return Poll::Ready(0);
            }
        }

        // println!("^^^ [{:x}] w poll pen ^^^^", self.driver.addr_no());
        // push_trace(ASYNC_WRITE_POLL | self.write_len);
        Poll::Pending
    }
}




struct Handler {
    driver: Arc<AsyncSerial>,
    irqn: u16
}

impl Handler {
    pub fn new(driver: Arc<AsyncSerial>, irqn: u16) -> Self {
        Handler {
            driver,
            irqn
        }
    }
    fn interrupt_handler(&self){
        // use crate::trace::{ASYNC_READ_WAKE, ASYNC_WRITE_WAKE};
        use core::sync::atomic::Ordering::{Acquire, Release};
        use uart::iir::Iid;

        let driver = &self.driver;
        let block = driver.hardware();
        while let Some(int_type) = block.iir().read().iid().variant() {
            // there is no Interrupt
            if int_type == Iid::NoInterruptPending {
                break;
            }
            // let intr_id: usize = int_type as u8 as _;
            // push_trace(SERIAL_INTR_ENTER + intr_id);
            // self.intr_count.fetch_add(1, Relaxed);
            match int_type {
                // there is some data to receive
                // 接收串口中的数据
                Iid::ReceivedDataAvailable | Iid::CharacterTimeout => {
                    // println!("[SERIAL] Received data available");
                    driver.serial_rx.rx_intr_count.fetch_add(1, Relaxed);
                    let mut rx_count = 0;
                    let mut rx_fifo_count = driver.serial_rx.rx_fifo_count.load(Acquire);
                    let mut pro = driver.serial_rx.rx_pro.lock();
                    while let Some(ch) = driver.try_recv() {
                        rx_fifo_count += 1;
                        rx_count += 1;
                        if rx_fifo_count == RTS_PULSE_WIDTH {
                            // push_trace(SERIAL_RTS);
                            driver.rts(false);
                        } else if rx_fifo_count == RTS_PULSE_WIDTH * 2 {
                            // push_trace(SERIAL_RTS | 1);
                            driver.rts(true);
                            rx_fifo_count = 0;
                        }
                        if let Err(_) = pro.enqueue(ch) {
                            // println!("[USER UART] Serial rx buffer overflow!");
                        }
                        if pro.len() >= DEFAULT_RX_BUFFER_SIZE - 1 {
                            driver.disable_rdai();
                            break;
                        }
                    }
                    driver.serial_rx.rx_fifo_count.store(rx_fifo_count, Release);
                    driver.serial_rx.rx_count.fetch_add(rx_count, Relaxed);
                    if let Some(waker) = driver.read_waker.try_lock() {
                        if waker.is_some() {
                            // println!("*** [{}] r wake ****", self.addr_no());
                            // waker.take().unwrap().wake();
                            // push_trace(ASYNC_READ_WAKE);
                            waker.as_ref().unwrap().wake_by_ref();
                        } else {
                            // println!("&&& [{}] no r waker &&&&", self.addr_no());
                        }
                    } else {
                        // println!("cannot lock reader waker");
                    }
                }
                // 
                Iid::ThrEmpty => {
                    // println!("[SERIAL] Transmitter Holding Register Empty");
                    driver.serial_tx.tx_intr_count.fetch_add(1, Relaxed);
                    driver.serial_tx.start_tx();
                }
                // 
                Iid::ReceiverLineStatus => {
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
                // 
                Iid::ModemStatus => {
                    if driver.dcts() {
                        let cts = driver.cts();
                        if cts == driver.serial_tx.prev_cts.load(Relaxed) {
                            // push_trace(SERIAL_CTS | (RTS_PULSE_WIDTH * 2));
                            driver.serial_tx.tx_fifo_count
                                .fetch_add(-(RTS_PULSE_WIDTH as isize * 2), Relaxed);
                        } else {
                            // push_trace(SERIAL_CTS | RTS_PULSE_WIDTH);
                            driver.serial_tx.tx_fifo_count
                                .fetch_add(-(RTS_PULSE_WIDTH as isize), Relaxed);
                        }
                        driver.serial_tx.prev_cts.store(cts, Relaxed);
                        driver.toggle_threi();
                        // println!("dcts && cts");
                        if let Some(waker) = self.driver.write_waker.try_lock() {
                            if waker.is_some() {
                                // println!("%%% [{}] w wake %%%%", self.addr_no());
                                // waker.take().unwrap().wake();
                                // push_trace(ASYNC_WRITE_WAKE);
                                waker.as_ref().unwrap().wake_by_ref();
                            } else {
                                // println!("___ [{}] no w waker ____", self.addr_no());
                            }
                        } else {
                            // println!("cannot lock writer waker");
                        }
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
                    // todo, has not been supported
                    // println!("[USER SERIAL] {:?} not supported!", int_type);
                }
            }

        }
    }
}

impl Future for Handler {
    type Output = i32;
    fn poll(mut self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<Self::Output> {
        self.interrupt_handler();
        Poll::Pending
    }
}

unsafe impl Send for Handler {}
unsafe impl Sync for Handler {}

