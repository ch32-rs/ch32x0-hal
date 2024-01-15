//! Simple busy-loop delay provider

use fugit::HertzU32;
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

/// A delay provided by the SysTick core peripheral
///
/// This requires SysTick to be set up and running.
/// Assumes conditions: upcount, hclk/8
pub struct SystickDelay;

impl SystickDelay {
    /// Init Systick.
    ///
    /// # Safety
    ///
    /// This function is unsafe because it conflicts with embassy's systick time dirver.
    /// Only one of them can be used.
    pub unsafe fn init() {
        let rb = &*pac::SYSTICK::PTR;
        rb.ctlr().modify(|_, w| {
            w.init()
                .set_bit()
                .mode()
                .upcount()
                .stre()
                .clear_bit()
                .stclk()
                .hclk_div8()
                .ste()
                .set_bit()
        });
    }
}

impl embedded_hal::delay::DelayNs for SystickDelay {
    fn delay_ns(&mut self, ns: u32) {
        let rb = unsafe { &*pac::SYSTICK::PTR };

        let ticks = ns as u64 * clocks().sysclk.to_Hz() as u64 / 8 / 1_000_000_000;
        let target = rb.cnt().read().bits().wrapping_add(ticks);

        while rb.cnt().read().bits() < target {}
    }
    fn delay_us(&mut self, us: u32) {
        let rb = unsafe { &*pac::SYSTICK::PTR };

        let ticks = us as u64 * clocks().sysclk.to_Hz() as u64 / 8 / 1_000_000;
        let target = rb.cnt().read().bits().wrapping_add(ticks);

        while rb.cnt().read().bits() < target {}
    }
    fn delay_ms(&mut self, mut ms: u32) {
        let rb = unsafe { &*pac::SYSTICK::PTR };

        let ticks = ms as u64 * clocks().sysclk.to_Hz() as u64 / 8 / 1_000;
        let target = rb.cnt().read().bits().wrapping_add(ticks);

        while ms > 0 {
            while rb.cnt().read().bits() < target {}

            ms -= 1;
        }
    }
}
