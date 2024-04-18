#![no_std]

extern crate alloc;



pub mod serials;
pub mod future;
mod task;
mod waker;

pub const TEST: usize = 1;

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(any(feature = "board_qemu", feature = "board_lrv"))]
    #[test]
    fn intro_uart(){
        use serials;
        assert!(true);
        assert_eq!(TEST, 1);
    }

    #[cfg(any(feature = "board_qemu", feature = "board_lrv"))]
    #[test]
    fn sync_uart_test(){
        use serials::BufferedSerial;
        const BASE_ADDRESS: usize = 0x1000;
        // const BAUD_RATE: usize = 9600;
        // const BAUD_RATE: usize = 115_200;
        // const BAUD_RATE: usize = 921_600;
        // const BAUD_RATE: usize = 1_250_000;
        const BAUD_RATE: usize = 6_250_000;

        let mut hardware = BufferedSerial::new(BASE_ADDRESS);
        hardware.hardware_init(BAUD_RATE);
    }
}