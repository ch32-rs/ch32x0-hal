use embedded_hal::delay::DelayNs;

use crate::gpio::Pin;
use crate::{into_ref, peripherals, Peripheral};

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
}

impl Default for Config {
    fn default() -> Self {
        Self {
            // Use Div6 as default
            // Power on default is Div4,
            // Ofiicial SDK is Div6
            clkdiv: 0b0101,
        }
    }
}

/// Analog to Digital driver.
pub struct Adc<'d, T: Instance> {
    #[allow(unused)]
    adc: crate::PeripheralRef<'d, T>,
}

impl<'d, T: Instance> Adc<'d, T> {
    pub fn new(adc: impl Peripheral<P = T> + 'd, config: Config) -> Self {
        into_ref!(adc);
        T::enable_and_reset();

        T::regs().ctlr3().modify(|w| w.set_clk_div(config.clkdiv));

        // clear, only independent mode is supported
        T::regs().ctlr1().modify(|w| {
            w.0 = 0;
        });

        // no external trigger
        // no continuous mode
        T::regs().ctlr2().modify(|w| {
            w.set_align(false); // right aligned
            w.set_extsel(0b111); //  SWSTART Software trigger
            w.set_cont(false); // single conversion
        });

        const CHANNEL_COUNT: u8 = 1;
        T::regs().rsqr1().modify(|w| w.set_l(CHANNEL_COUNT - 1));

        // ADC ON
        T::regs().ctlr2().modify(|w| w.set_adon(true));

        Self { adc }
    }

    // regular conversion
    fn configure_channel(&mut self, channel: &mut impl AdcPin<T>, rank: u8, sample_time: SampleTime) {
        channel.set_as_analog();

        let channel = channel.channel();

        // sample time config
        let bits = sample_time as u8;
        if channel < 10 {
            T::regs().samptr2().modify(|w| w.set_smp(channel as usize, bits));
        } else {
            T::regs().samptr1().modify(|w| w.set_smp((channel - 10) as usize, bits));
        }

        // regular sequence config
        assert!(rank < 17 || rank > 0);
        if rank < 7 {
            T::regs()
                .rsqr3()
                .modify(|w| w.set_sq((rank - 1) as usize, channel & 0b11111));
        } else if rank < 13 {
            T::regs()
                .rsqr2()
                .modify(|w| w.set_sq((rank - 7) as usize, channel & 0b11111));
        } else {
            T::regs()
                .rsqr1()
                .modify(|w| w.set_sq((rank - 13) as usize, channel & 0b11111));
        }
    }

    // Get_ADC_Val
    // FIXME: channel is not used
    pub fn convert(&mut self, channel: &mut impl AdcPin<T>, sample_time: SampleTime) -> u16 {
        self.configure_channel(channel, 1, sample_time);

        T::regs().ctlr2().modify(|w| w.set_swstart(true));

        // while not end of conversion
        while !T::regs().statr().read().eoc() {
            core::hint::spin_loop();
        }

        T::regs().rdatar().read().data()
    }
}

pub(crate) mod sealed {

    //pub trait InterruptableInstance {
    //        type Interrupt: crate::interrupt::Interrupt;
    //  }
    //InterruptableInstance

    pub trait Instance {
        fn regs() -> &'static crate::pac::adc::Adc;

        fn enable_and_reset() {
            let rcc = &crate::pac::RCC;
            rcc.apb2pcenr().modify(|w| w.set_adc1en(true));
            rcc.apb2prstr().modify(|w| w.set_adc1rst(true));
            rcc.apb2prstr().modify(|w| w.set_adc1rst(false));
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

impl sealed::Instance for peripherals::ADC1 {
    fn regs() -> &'static crate::pac::adc::Adc {
        &crate::pac::ADC1
    }
}

impl Instance for peripherals::ADC1 {}

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

impl_adc_pin!(ADC1, PA0, 0);
impl_adc_pin!(ADC1, PA1, 1);
impl_adc_pin!(ADC1, PA2, 2);
impl_adc_pin!(ADC1, PA3, 3);
impl_adc_pin!(ADC1, PA4, 4);
impl_adc_pin!(ADC1, PA5, 5);
impl_adc_pin!(ADC1, PA6, 6);
impl_adc_pin!(ADC1, PA7, 7);
impl_adc_pin!(ADC1, PB0, 8);
impl_adc_pin!(ADC1, PB1, 9);
impl_adc_pin!(ADC1, PC0, 10);
impl_adc_pin!(ADC1, PC1, 11);
impl_adc_pin!(ADC1, PC2, 12);
impl_adc_pin!(ADC1, PC3, 13);

// pub struct Vref;
//impl<T: Instance> AdcPin<T> for Vref {}
//impl<T: Instance> sealed::AdcPin<T> for Vref {
//    fn channel(&self) -> u8 {
//        15
//    }
//}
