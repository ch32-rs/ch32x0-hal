//! Op-Amp, OPA1 and OPA2

/*
OPA1 and OPA2 shares the same register block `OPA`.
This mod splits it into to virtual peripherals.


OPA输入引脚或通道可选择
OPA输出引脚可选择通用I/O口或ADC采样通道
OPA支持正端输入轮询功能
OPA支持PGA增益选择
CMP输入引脚可选择 负端输入通道可选公用引脚
CMP输出引脚可选择通用I/O口或TIM内部采样通道
1个中断向量
*/
use crate::gpio::Pull;
use crate::{into_ref, pac, peripherals, Peripheral, PeripheralRef};

/// Gain for no external inverting input(wired to GND)
#[allow(missing_docs)]
#[derive(Clone, Copy)]
pub enum OpAmpGain {
    /// 64k
    Mul4 = 0b011,
    /// 27.4k
    Mul8 = 0b100,
    /// 12.8k
    Mul16 = 0b101,
    /// 6.2k
    Mul32 = 0b110,
}

/// OpAmp external outputs, wired to a GPIO pad.
///
/// This struct can also be used as an ADC input.
pub struct OpAmpOutput<'d, T: Instance> {
    _inner: &'d OpAmp<'d, T>,
}

/// OpAmp internal outputs, wired directly to ADC inputs.
///
/// This struct can be used as an ADC input.
pub struct OpAmpInternalOutput<'d, T: Instance> {
    _inner: &'d OpAmp<'d, T>,
}

pub struct OpAmp<'d, T: Instance> {
    _inner: PeripheralRef<'d, T>,
}

impl<'d, T: Instance> OpAmp<'d, T> {
    /// Create a new driver instance.
    pub fn new(opamp: impl Peripheral<P = T> + 'd) -> Self {
        into_ref!(opamp);

        Self { _inner: opamp }
    }

    /// Non-inverting Operational Amplifier
    pub fn buffer_non_inverting(
        &'d mut self,
        in_pin: impl Peripheral<P = impl NonInvertingPin<T> + crate::gpio::sealed::Pin>,
        out_pin: impl Peripheral<P = impl OutputPin<T> + crate::gpio::sealed::Pin>, //  + 'd,
        gain: OpAmpGain,
    ) -> OpAmpOutput<'d, T> {
        into_ref!(in_pin);
        into_ref!(out_pin);

        in_pin.set_as_input(Pull::None);
        out_pin.set_as_analog();

        T::setup_input_output(in_pin.raw(), gain as u8, true, out_pin.raw());

        T::set_enable(true);

        OpAmpOutput { _inner: self }
    }

    /// For PA1, this has a gain of 16,
    pub fn buffer(
        &'d mut self,
        in_pin_positive: impl Peripheral<P = impl NonInvertingPin<T> + crate::gpio::sealed::Pin>,
        in_pin_negative: impl Peripheral<P = impl InvertingPin<T> + crate::gpio::sealed::Pin>,
        out_pin: impl Peripheral<P = impl OutputPin<T> + crate::gpio::sealed::Pin> + 'd,
    ) -> OpAmpOutput<'d, T> {
        into_ref!(in_pin_positive);
        into_ref!(in_pin_negative);
        into_ref!(out_pin);

        in_pin_positive.set_as_input(Pull::None);
        in_pin_negative.set_as_input(Pull::None);
        out_pin.set_as_analog();

        T::setup_input_output(in_pin_positive.raw(), in_pin_negative.raw(), false, out_pin.raw());

        T::set_enable(true);

        OpAmpOutput { _inner: self }
    }
}

impl<'d, T: Instance> Drop for OpAmpOutput<'d, T> {
    fn drop(&mut self) {
        // T::set_enable(false);
    }
}

/// Opamp instance trait.
pub trait Instance: sealed::Instance + 'static {}

pub(crate) mod sealed {
    pub trait Instance {
        fn regs() -> &'static crate::pac::opa::Opa {
            &crate::pac::OPA
        }

        fn set_enable(enable: bool);

        fn setup_input_output(psel: u8, nsel: u8, enbale_fb: bool, outsel: u8);
    }

    pub trait NonInvertingPin<T: Instance> {
        fn raw(&self) -> u8;
    }

    pub trait InvertingPin<T: Instance> {
        fn raw(&self) -> u8;
    }

    pub trait OutputPin<T: Instance> {
        fn raw(&self) -> u8;
    }
}

/// Non-inverting pin trait.
pub trait NonInvertingPin<T: Instance>: sealed::NonInvertingPin<T> {}
/// Inverting pin trait.
pub trait InvertingPin<T: Instance>: sealed::InvertingPin<T> {}
/// Output pin trait.
pub trait OutputPin<T: Instance>: sealed::OutputPin<T> {}

impl Instance for peripherals::OPA1 {}
impl sealed::Instance for peripherals::OPA1 {
    fn set_enable(enable: bool) {
        Self::regs().ctlr1().modify(|w| w.set_en1(enable));
    }

    fn setup_input_output(psel: u8, nsel: u8, enbale_fb: bool, outsel: u8) {
        Self::regs().ctlr1().modify(|w| {
            w.set_psel1(psel);
            w.set_nsel1(nsel);
            w.set_fb_en1(enbale_fb);
            w.set_mode1(outsel != 0);
        });
    }
}

impl Instance for peripherals::OPA2 {}
impl sealed::Instance for peripherals::OPA2 {
    fn set_enable(enable: bool) {
        Self::regs().ctlr1().modify(|w| w.set_en2(enable));
    }

    fn setup_input_output(psel: u8, nsel: u8, enbale_fb: bool, outsel: u8) {
        Self::regs().ctlr1().modify(|w| {
            w.set_psel2(psel);
            w.set_nsel2(nsel);
            w.set_fb_en2(enbale_fb);
            w.set_mode2(outsel != 0);
        });
    }
}

#[allow(unused_macros)]
macro_rules! impl_opa_pin {
    ($inst:ident, $pin_kind:ident, $pin:ident, $raw_reg_val:expr) => {
        impl crate::opa::$pin_kind<peripherals::$inst> for crate::peripherals::$pin {}
        impl crate::opa::sealed::$pin_kind<peripherals::$inst> for crate::peripherals::$pin {
            fn raw(&self) -> u8 {
                $raw_reg_val
            }
        }
    };
}

impl_opa_pin!(OPA1, NonInvertingPin, PB0, 0b00);
impl_opa_pin!(OPA1, NonInvertingPin, PB8, 0b01);
impl_opa_pin!(OPA1, NonInvertingPin, PB4, 0b10);

impl_opa_pin!(OPA1, InvertingPin, PA6, 0b000);
impl_opa_pin!(OPA1, InvertingPin, PB6, 0b001);
impl_opa_pin!(OPA1, InvertingPin, PA1, 0b010);

impl_opa_pin!(OPA1, OutputPin, PA3, 0);
impl_opa_pin!(OPA1, OutputPin, PB5, 1);

impl_opa_pin!(OPA2, NonInvertingPin, PA7, 0b00);
impl_opa_pin!(OPA2, NonInvertingPin, PB3, 0b01);
impl_opa_pin!(OPA2, NonInvertingPin, PB7, 0b10);

impl_opa_pin!(OPA2, InvertingPin, PA5, 0b000);
impl_opa_pin!(OPA2, InvertingPin, PB1, 0b001);
impl_opa_pin!(OPA2, InvertingPin, PA1, 0b010);

impl_opa_pin!(OPA2, OutputPin, PA4, 0);
impl_opa_pin!(OPA2, OutputPin, PA2, 1);

pub(crate) unsafe fn init() {
    // Unlock OPA
    let regs = &crate::pac::OPA;
    regs.opa_key().write(|w| w.0 = 0x45670123);
    regs.opa_key().write(|w| w.0 = 0xCDEF89AB);
}
