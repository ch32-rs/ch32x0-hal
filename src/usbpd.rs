use crate::gpio::sealed::Pin;
use crate::gpio::Pull;
use crate::{into_ref, pac, peripherals, Peripheral, PeripheralRef};

pub struct UsbPdSink<'d, T: Instance> {
    _peri: PeripheralRef<'d, T>,
}

impl<'d, T: Instance> UsbPdSink<'d, T> {
    /// Create a new SPI driver.
    pub fn new(
        _peri: impl Into<PeripheralRef<'d, T>>,
        cc1: impl Peripheral<P = impl Cc1Pin<T>> + 'd,
        cc2: impl Peripheral<P = impl Cc2Pin<T>> + 'd,
    ) -> Self {
        into_ref!(cc1, cc2);

        let afio = unsafe { &*pac::AFIO::PTR };

        cc1.set_as_input(Pull::None);
        cc2.set_as_input(Pull::None);

        afio.ctlr()
            .modify(|_, w| w.usbpd_in_hvt().set_bit().usbpd_phy_v33().set_bit());

        T::enable_and_reset();

        Self { _peri: _peri.into() }
    }

    pub fn detect_cc(&mut self) -> bool {
        false
    }
}

pub(crate) mod sealed {
    use super::*;

    pub trait Instance {
        #[inline(always)]
        fn regs() -> &'static pac::usbpd::RegisterBlock {
            unsafe { &*pac::USBPD::PTR }
        }

        fn enable_and_reset() {
            let rcc = unsafe { &*pac::RCC::ptr() };
            rcc.ahbpcenr().modify(|_, w| w.usbpd().set_bit());
            rcc.ahbrstr().modify(|_, w| w.usbpdrst().set_bit());
            rcc.ahbrstr().modify(|_, w| w.usbpdrst().clear_bit());
        }
    }
}

pub trait Instance: Peripheral<P = Self> + sealed::Instance {}

impl sealed::Instance for peripherals::USBPD {}
impl Instance for peripherals::USBPD {}

pub trait Cc1Pin<T: Instance>: crate::gpio::Pin {}
pub trait Cc2Pin<T: Instance>: crate::gpio::Pin {}

macro_rules! pin_trait_impl {
    (crate::$mod:ident::$trait:ident, $instance:ident, $pin:ident) => {
        impl crate::$mod::$trait<crate::peripherals::$instance> for crate::peripherals::$pin {}
    };
}

pin_trait_impl!(crate::usbpd::Cc1Pin, USBPD, PC14);
pin_trait_impl!(crate::usbpd::Cc2Pin, USBPD, PC15);
