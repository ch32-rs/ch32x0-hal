//! Simple busy-loop delay provider

use fugit::HertzU32;
use qingke::riscv;
use qingke_rt::highcode;

use crate::rcc::clocks;

/// A delay provided by busy-looping
pub struct CycleDelay;

impl embedded_hal::delay::DelayNs for CycleDelay {
    #[highcode]
    fn delay_ns(&mut self, ns: u32) {
        let cycles = ns as u64 * clocks().sysclk.to_Hz() as u64 / 1_500_000_000;

        unsafe {
            riscv::asm::delay(cycles as u32);
        }
    }
    #[highcode]
    fn delay_us(&mut self, us: u32) {
        let cycles = us as u64 * clocks().sysclk.to_Hz() as u64 / 1_500_000;

        unsafe {
            riscv::asm::delay(cycles as u32);
        }
    }

    #[highcode]
    fn delay_ms(&mut self, mut ms: u32) {
        let cycles_per_us = clocks().sysclk.to_Hz() as u32 / 1_500_000;

        for _ in 0..ms {
            unsafe {
                riscv::asm::delay(cycles_per_us * 1000);
            }
        }
    }
}
