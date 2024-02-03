use embedded_hal::delay::DelayNs;

use crate::delay::SystickDelay;
use crate::gpio::Pin;
use crate::{into_ref, peripherals, Peripheral, PeripheralRef};

pub const ADC_MAX: u32 = (1 << 12) - 1;
// No calibration data, voltage should be 1.2V (1.16 to 1.24)
pub const VREF_INT: u32 = 1200;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SampleTime {
    #[default]
    Cycles4 = 0b000,
    Cycles5 = 0b001,
    Cycles6 = 0b010,
    Cycles7 = 0b011,
    Cycles8 = 0b100,
    Cycles9 = 0b101,
    Cycles10 = 0b110,
    Cycles11 = 0b111,
}

pub struct Config {
    /// Div1 to Div16
    // raw values are 0 to 0b111
    pub clkdiv: u8,
    // TODO: handle "-1"
    pub channel_count: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            // Use Div6 as default
            // Power on default is Div4,
            // Ofiicial SDK is Div6
            clkdiv: 0b0101,
            channel_count: 1,
        }
    }
}

/// Analog to Digital driver.
pub struct Adc<'d, T: Instance> {
    #[allow(unused)]
    adc: crate::PeripheralRef<'d, T>,
}

impl<'d, T: Instance> Adc<'d, T> {
    pub fn new(adc: impl Peripheral<P = T> + 'd, delay: &mut impl DelayNs, config: Config) -> Self {
        into_ref!(adc);
        T::enable_and_reset();

        T::regs().ctlr3().modify(|_, w| w.clk_div().variant(config.clkdiv));

        T::regs().ctlr1().modify(|_, w| {
            unsafe { w.bits(0) } // clear, only independent mode is supported
        });

        // no external trigger
        // no continuous mode
        T::regs().ctlr2().modify(|_, w| {
            w.align()
                .clear_bit() // right aligned
                .extsel()
                .variant(0b111) //  SWSTART 软件触发
        });

        // ADC ON
        T::regs().ctlr2().modify(|_, w| w.adon().set_bit());
        delay.delay_us(1000);

        T::regs().rsqr1().modify(|_, w| w.l().variant(config.channel_count - 1));

        Self { adc }
    }

    // regular conversion
    pub fn configure_channel(&mut self, channel: &mut impl AdcPin<T>, rank: u8, sample_time: SampleTime) {
        channel.set_as_analog();

        let channel = channel.channel();

        // sample time config
        let bits = sample_time as u32;
        if channel > 9 {
            T::regs().samptr1_charge1().modify(|r, w| unsafe {
                w.bits((r.bits() & !(0b111 << (channel - 10) * 3)) | (bits << (channel - 10) * 3))
            });
        } else {
            T::regs()
                .samptr2_charge2()
                .modify(|r, w| unsafe { w.bits((r.bits() & !(0b111 << channel * 3)) | (bits << channel * 3)) });
        }

        // rank config
        assert!(rank < 17 || rank > 0);
        if rank < 7 {
            let offset = (rank - 1) * 5;
            T::regs()
                .rsqr3()
                .modify(|r, w| unsafe { w.bits(r.bits() & !(0b11111 << offset) | (channel as u32) << offset) })
        } else if rank < 13 {
            let offset = (rank - 7) * 5;
            T::regs()
                .rsqr2()
                .modify(|r, w| unsafe { w.bits(r.bits() & !(0b11111 << offset) | (channel as u32) << offset) })
        } else {
            let offset = (rank - 13) * 5;
            T::regs()
                .rsqr1()
                .modify(|r, w| unsafe { w.bits(r.bits() & !(0b11111 << offset) | (channel as u32) << offset) })
        }
    }

    // Get_ADC_Val
    pub fn convert(&mut self, channel: &mut impl AdcPin<T>, delay: &mut impl DelayNs) -> u16 {
        T::regs().ctlr2().modify(|_, w| w.swstart().set_bit());

        while T::regs().statr().read().eoc().bit_is_clear() {
            delay.delay_us(1);
        }

        (T::regs().rdatar_dr_act_dcg().read().bits() & 0xffff) as u16
    }
}

pub(crate) mod sealed {

    //pub trait InterruptableInstance {
    //        type Interrupt: crate::interrupt::Interrupt;
    //  }
    //InterruptableInstance

    pub trait Instance {
        fn regs() -> &'static crate::pac::adc::RegisterBlock;

        fn enable_and_reset() {
            let rcc = unsafe { &*crate::pac::RCC::ptr() };
            rcc.apb2pcenr().modify(|_, w| w.adc1en().set_bit());
            rcc.apb2prstr().modify(|_, w| w.adc1rst().set_bit());
            rcc.apb2prstr().modify(|_, w| w.adc1rst().clear_bit());
        }
    }

    pub trait AdcPin<T: Instance> {
        fn channel(&self) -> u8;
    }

    pub trait InternalChannel<T> {
        fn channel(&self) -> u8;
    }
}

pub trait Instance: sealed::Instance + crate::Peripheral<P = Self> {}

/// ADC pin.
pub trait AdcPin<T: Instance>: sealed::AdcPin<T> + Pin {}
/// ADC internal channel.
pub trait InternalChannel<T>: sealed::InternalChannel<T> {}

impl sealed::Instance for peripherals::ADC {
    fn regs() -> &'static crate::pac::adc::RegisterBlock {
        unsafe { &*crate::pac::ADC::PTR }
    }
}

impl Instance for peripherals::ADC {}

macro_rules! impl_adc_pin {
    ($inst:ident, $pin:ident, $ch:expr) => {
        impl crate::adc::AdcPin<peripherals::$inst> for crate::peripherals::$pin {}

        impl crate::adc::sealed::AdcPin<peripherals::$inst> for crate::peripherals::$pin {
            fn channel(&self) -> u8 {
                $ch
            }
        }
    };
}

impl_adc_pin!(ADC, PA0, 0);
impl_adc_pin!(ADC, PA1, 1);
impl_adc_pin!(ADC, PA2, 2);
impl_adc_pin!(ADC, PA3, 3);
impl_adc_pin!(ADC, PA4, 4);
impl_adc_pin!(ADC, PA5, 5);
impl_adc_pin!(ADC, PA6, 6);
impl_adc_pin!(ADC, PA7, 7);
impl_adc_pin!(ADC, PB0, 8);
impl_adc_pin!(ADC, PB1, 9);
impl_adc_pin!(ADC, PC0, 10);
impl_adc_pin!(ADC, PC1, 11);
impl_adc_pin!(ADC, PC2, 12);
impl_adc_pin!(ADC, PC3, 13);

// pub struct Vref;
//impl<T: Instance> AdcPin<T> for Vref {}
//impl<T: Instance> sealed::AdcPin<T> for Vref {
//    fn channel(&self) -> u8 {
//        15
//    }
//}
