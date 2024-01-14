use alloc::collections::VecDeque;
use embedded_hal::serial::{Read, Write};
use core::{convert::Infallible, pin::Pin, sync::atomic::AtomicBool};

#[cfg(feature= "board_lrv")]
use lrv_pac::uart;
#[cfg(feature = "board_qemu")]
use qemu_pac::uart;

pub const FIFO_DEPTH: usize = 16;
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
        block.lcr.write(|w| w.dlab().set_bit());
        #[cfg(board = "board_lrv")]
        {
            block
                .dll()
                .write(|w| unsafe { w.bits((divisor & 0b1111_1111) as u32) });
            block
                .dlh()
                .write(|w| unsafe { w.bits(((divisor >> 8) & 0b1111_1111) as u32) });
        }
        #[cfg(board = "board_qemu")]
        {
            block
                .dll()
                .write(|w| unsafe { w.bits((divisor & 0b1111_1111) as u8) });
            block
                .dlh()
                .write(|w| unsafe { w.bits(((divisor >> 8) & 0b1111_1111) as u8) });
        }

        block.lcr.write(|w| w.dlab().clear_bit());
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
        if block.lsr.read().dr().bit_is_set() {
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
        let _unused = block.msr.read().bits();
        let _unused = block.lsr.read().bits();
        block.lcr.reset();
        // No modem control
        block.mcr.reset();
        block.ier().reset();
        block.fcr().reset();

        // Enable DLAB and Set divisor
        self.set_divisor(100_000_000, baud_rate);
        // Disable DLAB and set word length 8 bits, no parity, 1 stop bit
        block
            .lcr
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
        self.hardware().mcr.read().rts().is_asserted()
    }

    #[inline]
    pub fn rts(&self, is_asserted: bool) {
        self.hardware().mcr.modify(|_, w| w.rts().bit(is_asserted))
    }

    #[inline]
    pub fn cts(&self) -> bool {
        self.hardware().msr.read().cts().bit()
    }

    #[inline]
    pub fn dcts(&self) -> bool {
        self.hardware().msr.read().dcts().bit()
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

    #[cfg(any(board = "board_qemu", board = "board_lrv"))]
    pub fn interrupt_handler(&mut self) {
        // println!("[SERIAL] Interrupt!");

        use uart::iir::IID_A;

        while let Some(int_type) = self.hardware().iir().read().iid().variant() {
            if int_type == IID_A::NO_INTERRUPT_PENDING {
                break;
            }
            let intr_id: usize = int_type as u8 as _;
            push_trace(SERIAL_INTR_ENTER + intr_id);
            self.intr_count += 1;
            match int_type {
                IID_A::RECEIVED_DATA_AVAILABLE | IID_A::CHARACTER_TIMEOUT => {
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
                IID_A::THR_EMPTY => {
                    self.tx_intr_count += 1;
                    // println!("[SERIAL] Transmitter Holding Register Empty");
                    self.start_tx();
                }
                IID_A::RECEIVER_LINE_STATUS => {
                    let block = self.hardware();
                    let lsr = block.lsr.read();
                    // if lsr.bi().bit_is_set() {
                    if lsr.fifoerr().is_error() {
                        if lsr.bi().bit_is_set() {
                            println!("[uart] lsr.BI!");
                        }
                        if lsr.fe().bit_is_set() {
                            println!("[uart] lsr.FE!");
                        }
                        if lsr.pe().bit_is_set() {
                            println!("[uart] lsr.PE!");
                        }
                    }
                    if lsr.oe().bit_is_set() {
                        block.mcr.modify(|_, w| w.rts().deasserted());
                        println!("[uart] lsr.OE!");
                    }
                }
                IID_A::MODEM_STATUS => {
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
                        println!(
                            "[USER SERIAL] EDSSI, MSR: {:#x}, LSR: {:#x}, IER: {:#x}",
                            block.msr.read().bits(),
                            block.lsr.read().bits(),
                            block.ier().read().bits()
                        );
                    }
                }
                _ => {
                    println!("[USER SERIAL] {:?} not supported!", int_type);
                }
            }
            push_trace(SERIAL_INTR_EXIT + intr_id);
        }
    }
}

impl Write<u8> for BufferedSerial {
    type Error = Infallible;

    #[cfg(any(feature = "board_qemu", feature = "board_lrv"))]
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
        let _unused = block.msr.read().bits();
        let _unused = block.lsr.read().bits();
        self.rts(false);
        // reset Rx & Tx FIFO, disable FIFO
        block
            .fcr()
            .write(|w| w.fifoe().clear_bit().rfifor().set_bit().xfifor().set_bit());
    }
}
