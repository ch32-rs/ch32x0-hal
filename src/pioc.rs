//! PIOC
//!
//! - Running at HCLK
//! - Support interrupt, number 47
//! - PC18 PIOC_IO0, PC19 PIOC_IO1
//! - alt for >= 48 pins
//!   - PC7 PIOC_IO0_1 ???

use crate::gpio::sealed::Pin;
use crate::{interrupt, into_ref, pac, peripherals, Peripheral, PeripheralRef};

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: core::marker::PhantomData<T>,
}

impl<T: Instance> interrupt::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        // TODO
    }
}

/// PIOC driver
pub struct Pioc<'d, T: Instance> {
    _peri: PeripheralRef<'d, T>,
}

impl<'d, T: Instance> Pioc<'d, T> {
    pub const DATA: &'static [u8; ]

    /// Create a new driver instance.
    pub fn new<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        pio0: impl Peripheral<P = impl Io0Pin<T, REMAP>> + 'd,
        pio1: impl Peripheral<P = impl Io1Pin<T, REMAP>> + 'd,
    ) -> Self {
        into_ref!(peri, pio0, pio1);

        T::enable_and_reset();

        pio0.set_as_af_output();
        pio1.set_as_af_output();

        Self { _peri: peri }
    }

    pub fn new_pio0<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        pio0: impl Peripheral<P = impl Io0Pin<T, REMAP>> + 'd,
    ) -> Self {
        into_ref!(peri, pio0);

        T::enable_and_reset();

        pio0.set_as_af_output();

        Self { _peri: peri }
    }

    pub fn new_pio1<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        pio1: impl Peripheral<P = impl Io1Pin<T, REMAP>> + 'd,
    ) -> Self {
        into_ref!(peri, pio1);

        T::enable_and_reset();

        pio1.set_as_af_output();

        Self { _peri: peri }
    }

    pub fn load_and_run(&self, program: &[u8]) {
        const PIOC_SRAM_BASE: *mut u8 = 0x20004000 as _;
        unsafe {
            core::ptr::copy_nonoverlapping(program.as_ptr(), PIOC_SRAM_BASE, program.len());
        }

        let regs = T::regs();
        regs.sys_cfg()
            .modify(|_, w| w.reset().set_bit().io_en0().set_bit().io_en1().set_bit());
        regs.sys_cfg().modify(|_, w| w.reset().clear_bit());
        regs.sys_cfg().modify(|_, w| w.clk_gate().set_bit());
    }

    #[inline]
    pub fn data(&self) -> &mut [u8; 32] {
        unsafe { &mut *((0x40026C00 +0x20)  as *mut [u8; 32]) }
    }

    #[inline]
    pub fn data_exch(&self) -> &mut u8 {
        unsafe { &mut *((0x40026C00 +0x1F)  as *mut u8) }
    }

    #[inline]
    pub fn wr(&self) -> &mut u8 {
        unsafe { &mut *((0x40026C00 +0x1E)  as *mut u8) }
    }


}

pub(crate) mod sealed {
    use super::*;

    pub trait Instance {
        fn regs() -> &'static pac::pioc::RegisterBlock {
            unsafe { &*pac::PIOC::ptr() }
        }

        fn enable_and_reset() {
            // PIOC is always enable
            // prerequirement: GPIOC and AFIO
        }

        fn set_remap(remap: u8) {
            let afio = unsafe { &*pac::AFIO::ptr() };
            afio.pcfr1().modify(|_, w| w.ploc_rm().bit(remap != 0));
        }
    }
}

pub trait Instance: sealed::Instance + 'static {
    type Interrupt: interrupt::Interrupt;
}

impl sealed::Instance for peripherals::PIOC {}
impl Instance for peripherals::PIOC {
    type Interrupt = interrupt::PIOC;
}

macro_rules! pin_trait {
    ($signal:ident, $instance:path) => {
        pub trait $signal<T: $instance, const REMAP: u8>: crate::gpio::Pin {}
    };
}

pin_trait!(Io0Pin, Instance);
pin_trait!(Io1Pin, Instance);

macro_rules! pioc_pin_trait_impl {
    (crate::$mod:ident::$trait:ident, $instance:ident, $pin:ident, $remap:expr) => {
        impl crate::$mod::$trait<crate::peripherals::$instance, $remap> for crate::peripherals::$pin {}
    };
}

pioc_pin_trait_impl!(crate::pioc::Io0Pin, PIOC, PC18, 0);
pioc_pin_trait_impl!(crate::pioc::Io0Pin, PIOC, PC19, 0);

pioc_pin_trait_impl!(crate::pioc::Io0Pin, PIOC, PC7, 1);
pioc_pin_trait_impl!(crate::pioc::Io0Pin, PIOC, PC19, 1);
