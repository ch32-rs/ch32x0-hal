//! Simple busy-loop delay provider

use fugit::HertzU32;
use qingke::riscv;

use crate::rcc::clocks;

pub struct CycleDelay;

impl embedded_hal_1::delay::DelayNs for CycleDelay {
    fn delay_us(&mut self, us: u32) {
        let cycles = us as u64 * clocks().sysclk.to_Hz() as u64 / 8_000_000;

        unsafe {
            riscv::asm::delay(cycles as u32);
        }
    }

    fn delay_ns(&mut self, ns: u32) {
        let cycles = ns as u64 * clocks().sysclk.to_Hz() as u64 / 8_000_000_000;

        unsafe {
            riscv::asm::delay(cycles as u32);
        }
    }

    fn delay_ms(&mut self, mut ms: u32) {
        let cycles = 1000 * clocks().sysclk.to_Hz() as u64 / 8_000_000;

        while ms > 0 {
            unsafe {
                riscv::asm::delay(cycles as u32);
            }
            ms -= 1;
        }
    }
}
