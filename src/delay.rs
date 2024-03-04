//! Simple busy-loop delay provider

use pac::systick::vals;
use qingke::riscv;
use qingke_rt::highcode;

use crate::pac;
use crate::rcc::clocks;

/// A delay provided by busy-looping
pub struct CycleDelay;

impl embedded_hal::delay::DelayNs for CycleDelay {
    #[highcode]
    fn delay_ns(&mut self, ns: u32) {
        let cycles = ns as u64 * clocks().sysclk.to_Hz() as u64 / 1_500_000_000;

        riscv::asm::delay(cycles as u32);
    }
    #[highcode]
    fn delay_us(&mut self, us: u32) {
        let cycles = us as u64 * clocks().sysclk.to_Hz() as u64 / 1_500_000;

        riscv::asm::delay(cycles as u32);
    }

    #[highcode]
    fn delay_ms(&mut self, ms: u32) {
        let cycles_per_us = clocks().sysclk.to_Hz() as u32 / 1_500_000;

        for _ in 0..ms {
            riscv::asm::delay(cycles_per_us * 1000);
        }
    }
}

/// A delay provided by the SysTick core peripheral
///
/// This requires SysTick to be set up and running.
/// Assumes conditions: upcount, hclk.
/// hclk/8 is not accurate enough for ns delays.
pub struct SystickDelay;

impl SystickDelay {
    /// Init Systick.
    ///
    /// # Safety
    ///
    /// This function is unsafe because it conflicts with embassy's systick time dirver.
    /// Only one of them can be used.
    pub unsafe fn init() {
        let rb = &crate::pac::SYSTICK;
        rb.ctlr().modify(|w| {
            w.set_init(true);
            w.set_mode(vals::Mode::UPCOUNT);
            w.set_stre(false);
            w.set_stclk(vals::Stclk::HCLK);
            w.set_ste(true);
        });
    }

    // #[highcode]
    pub fn delay_ticks(&mut self, n: u32) {
        let rb = &crate::pac::SYSTICK;
        let target = rb.cntl().read().wrapping_add(n - 5); // 5 opcodes overhead

        // FIXME: handle overflow
        while rb.cntl().read() < target {}
    }
}

impl embedded_hal::delay::DelayNs for SystickDelay {
    fn delay_ns(&mut self, ns: u32) {
        let rb = &crate::pac::SYSTICK;

        let ticks = ns as u64 * clocks().sysclk.to_Hz() as u64 / 1_000_000_000;
        let target = rb.cnt().read().wrapping_add(ticks);

        while rb.cnt().read() < target {}
    }
    fn delay_us(&mut self, us: u32) {
        let rb = &crate::pac::SYSTICK;

        let ticks = us as u64 * clocks().sysclk.to_Hz() as u64 / 1_000_000;
        let target = rb.cnt().read().wrapping_add(ticks);

        while rb.cnt().read() < target {}
    }
    fn delay_ms(&mut self, mut ms: u32) {
        let rb = &crate::pac::SYSTICK;

        let ticks = ms as u64 * clocks().sysclk.to_Hz() as u64 / 1_000;
        let target = rb.cnt().read().wrapping_add(ticks);

        while ms > 0 {
            while rb.cnt().read() < target {}

            ms -= 1;
        }
    }
}
