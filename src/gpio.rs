//! GPIO
//!
//! 浮空输入
//! 上拉输入
//! 下拉输入(部分 IO)
//! 模拟输入
//! 推挽输出
//! 复用功能的输入和输出

use core::convert::Infallible;

use crate::{exti, impl_peripheral, into_ref, pac, peripherals, Peripheral, PeripheralRef};

/// GPIO flexible pin.
///
/// This pin can either be a disconnected, input, or output pin, or both. The level register bit will remain
/// set while not in output mode, so the pin's level will be 'remembered' when it is not in output
/// mode.
pub struct Flex<'d, T: Pin> {
    pub(crate) pin: PeripheralRef<'d, T>,
}

impl<'d, T: Pin> Flex<'d, T> {
    /// Wrap the pin in a `Flex`.
    ///
    /// The pin remains disconnected. The initial output level is unspecified, but can be changed
    /// before the pin is put into output mode.
    ///
    #[inline]
    pub fn new(pin: impl Peripheral<P = T> + 'd) -> Self {
        into_ref!(pin);
        // Pin will be in disconnected state.
        Self { pin }
    }

    #[inline]
    pub fn degrade(self) -> Flex<'d, AnyPin> {
        // Safety: We are about to drop the other copy of this pin, so
        // this clone is safe.
        let pin = unsafe { self.pin.clone_unchecked() };

        // We don't want to run the destructor here, because that would
        // deconfigure the pin.
        core::mem::forget(self);

        Flex {
            pin: pin.map_into::<AnyPin>(),
        }
    }

    /// Put the pin into input mode.
    #[inline]
    pub fn set_as_input(&mut self, pull: Pull) {
        critical_section::with(|_| {
            self.pin.set_as_input(pull);
        });
    }

    /// Put the pin into output mode.
    ///
    /// The pin level will be whatever was set before (or low by default). If you want it to begin
    /// at a specific level, call `set_high`/`set_low` on the pin first.
    #[inline]
    pub fn set_as_output(&mut self) {
        critical_section::with(|_| {
            self.pin.set_as_output();
        });
    }

    #[inline]
    pub fn is_high(&self) -> bool {
        !self.is_low()
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        self.pin.block().indr().read().idr(1 << self.pin.pin())
    }

    #[inline]
    pub fn get_level(&self) -> Level {
        self.is_high().into()
    }

    #[inline]
    pub fn is_set_high(&self) -> bool {
        !self.is_set_low()
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.pin.block().outdr().read().odr(1 << self.pin.pin())
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.is_set_high().into()
    }

    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_high();
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_low();
    }

    #[inline]
    pub fn set_level(&mut self, level: Level) {
        match level {
            Level::Low => self.pin.set_low(),
            Level::High => self.pin.set_high(),
        }
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        if self.is_set_low() {
            self.set_high()
        } else {
            self.set_low()
        }
    }
}

/// Pull setting for an input.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Pull {
    None,
    Up,
    Down,
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Level {
    Low,
    High,
}

impl From<bool> for Level {
    fn from(val: bool) -> Self {
        match val {
            true => Self::High,
            false => Self::Low,
        }
    }
}

impl From<u8> for Level {
    fn from(val: u8) -> Self {
        match val {
            0 => Self::Low,
            _ => Self::High,
        }
    }
}

impl From<Level> for bool {
    fn from(level: Level) -> bool {
        match level {
            Level::Low => false,
            Level::High => true,
        }
    }
}

/// GPIO input driver.
pub struct Input<'d, T: Pin> {
    pub(crate) pin: Flex<'d, T>,
}

impl<'d, T: Pin> Input<'d, T> {
    #[inline]
    pub fn new(pin: impl Peripheral<P = T> + 'd, pull: Pull) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_as_input(pull);
        Self { pin }
    }

    #[inline]
    pub fn degrade(self) -> Input<'d, AnyPin> {
        Input {
            pin: self.pin.degrade(),
        }
    }

    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }

    #[inline]
    pub fn get_level(&self) -> Level {
        self.pin.get_level()
    }
}

/// GPIO output driver.
///
/// Note that pins will **return to their floating state** when `Output` is dropped.
/// If pins should retain their state indefinitely, either keep ownership of the
/// `Output`, or pass it to [`core::mem::forget`].
pub struct Output<'d, T: Pin> {
    pub(crate) pin: Flex<'d, T>,
}

impl<'d, T: Pin> Output<'d, T> {
    #[inline]
    pub fn new(pin: impl Peripheral<P = T> + 'd, initial_output: Level) -> Self {
        let mut pin = Flex::new(pin);
        match initial_output {
            Level::High => pin.set_high(),
            Level::Low => pin.set_low(),
        }
        pin.set_as_output();
        Self { pin }
    }

    #[inline]
    pub fn degrade(self) -> Output<'d, AnyPin> {
        Output {
            pin: self.pin.degrade(),
        }
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_high();
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_low();
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_level(level)
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high()
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.pin.is_set_low()
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.get_output_level()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        self.pin.toggle();
    }
}

// NOTE: no OutputOpenDrain support

pub(crate) mod sealed {
    use super::*;

    pub trait Pin {
        fn pin_port(&self) -> u8;

        #[inline]
        fn _pin(&self) -> u8 {
            self.pin_port() % 32
        }
        #[inline]
        fn _port(&self) -> u8 {
            self.pin_port() / 32
        }

        #[inline]
        fn block(&self) -> pac::gpio::Gpio {
            crate::pac::GPIO(self._port() as usize)
        }

        /// Set the output as high.
        #[inline]
        fn set_high(&self) {
            let n = self._pin() as usize;
            if n < 16 {
                self.block().bshr().write(|w| w.set_bs(n, true));
            } else {
                self.block().bsxr().write(|w| w.set_bs(n - 16, true));
            }
        }

        /// Set the output as low.
        #[inline]
        fn set_low(&self) {
            let n = self._pin() as usize;
            if n < 16 {
                self.block().bshr().write(|w| w.set_br(n, true));
            } else {
                self.block().bsxr().write(|w| w.set_br(n - 16, true));
            }
        }

        #[inline]
        fn set_as_output(&self) {
            let cnf = 0b00;
            let mode = 0b10;

            self.set_cnf_mode(cnf, mode);
        }

        #[inline]
        fn set_as_input(&self, pull: Pull) {
            let cnf = if pull == Pull::None { 0b10 } else { 0b01 };
            let mode = 0b00;

            self.set_cnf_mode(cnf, mode);

            self.set_pull(pull);
        }

        #[inline]
        fn set_pull(&self, pull: Pull) {
            let pin = self._pin() as usize;
            let block = self.block();

            // Only PA0--PA15 and PC16--PC17 support input pull-down

            match pull {
                Pull::Up => block.outdr().modify(|w| {
                    w.set_odr(pin, true);
                }),
                Pull::Down => block.outdr().modify(|w| {
                    w.set_odr(pin, false);
                }),
                _ => {}
            }
        }

        /// Only one type, alternate function + push pull
        #[inline]
        fn set_as_af_output(&self) {
            let cnf = 0b10;
            let mode = 0b01;

            self.set_cnf_mode(cnf, mode);
        }

        #[inline]
        fn set_as_analog(&self) {
            let cnf = 0b00;
            let mode = 0b00;

            self.set_cnf_mode(cnf, mode);
        }

        /// Set the pin as "disconnected", ie doing nothing and consuming the lowest
        /// amount of power possible.
        ///
        /// This is currently the same as set_as_analog but is semantically different really.
        /// Drivers should set_as_disconnected pins when dropped.
        #[inline]
        fn set_as_disconnected(&self) {
            self.set_as_analog();
        }

        /// MODE=00, input
        /// CNF=00, analog
        /// CNF=01, floating input
        /// CNF=10, input with pull-up / pull-down
        /// CNF=11, reserved
        /// MODE=01, output
        /// CNF=00, general purpose output push-pull
        /// CNF=10, alternate function output Push-pull
        #[inline]
        fn set_cnf_mode(&self, cnf: u8, mode: u8) {
            let pin = self._pin() as usize;
            let block = self.block();

            let cnf = cnf & 0b11;
            let mode = mode & 0b11;

            match pin / 8 {
                0 => {
                    block.cfglr().modify(|w| {
                        w.set_cnf(pin % 8, cnf);
                        w.set_mode(pin % 8, mode);
                    });
                }
                1 => {
                    block.cfghr().modify(|w| {
                        w.set_cnf(pin % 8, cnf);
                        w.set_mode(pin % 8, mode);
                    });
                }
                2 => {
                    block.cfgxr().modify(|w| {
                        w.set_cnf(pin % 8, cnf);
                        w.set_mode(pin % 8, mode);
                    });
                }
                _ => unreachable!(),
            }
        }
    }
}

pub trait Pin: Peripheral<P = Self> + Into<AnyPin> + sealed::Pin + Sized + 'static {
    type ExtiChannel: crate::exti::Channel;

    /// Number of the pin within the port (0..31)
    #[inline]
    fn pin(&self) -> u8 {
        self._pin()
    }

    /// Port of the pin
    #[inline]
    fn port(&self) -> u8 {
        self._port()
    }

    /// Convert from concrete pin type PX_XX to type erased `AnyPin`.
    #[inline]
    fn degrade(self) -> AnyPin {
        AnyPin {
            pin_port: self.pin_port(),
        }
    }
}

// Type-erased GPIO pin
pub struct AnyPin {
    pin_port: u8,
}

impl AnyPin {
    #[inline]
    pub unsafe fn steal(pin_port: u8) -> Self {
        Self { pin_port }
    }

    #[inline]
    fn _port(&self) -> u8 {
        self.pin_port / 16
    }
}

impl_peripheral!(AnyPin);
impl Pin for AnyPin {
    type ExtiChannel = exti::AnyChannel;
}
impl sealed::Pin for AnyPin {
    #[inline]
    fn pin_port(&self) -> u8 {
        self.pin_port
    }
}

macro_rules! foreach_pin {
    ($($pat:tt => $code:tt;)*) => {
        macro_rules! __foreach_pin_inner {
            $(($pat) => $code;)*
            ($_:tt) => {}
        }
        __foreach_pin_inner!((PA0,GPIOA,0,0, EXTI0));
        __foreach_pin_inner!((PA1,GPIOA,0,1, EXTI1));
        __foreach_pin_inner!((PA2,GPIOA,0,2, EXTI2));
        __foreach_pin_inner!((PA3,GPIOA,0,3, EXTI3));
        __foreach_pin_inner!((PA4,GPIOA,0,4, EXTI4));
        __foreach_pin_inner!((PA5,GPIOA,0,5, EXTI5));
        __foreach_pin_inner!((PA6,GPIOA,0,6, EXTI6));
        __foreach_pin_inner!((PA7,GPIOA,0,7, EXTI7));
        __foreach_pin_inner!((PA8,GPIOA,0,8, EXTI8));
        __foreach_pin_inner!((PA9,GPIOA,0,9, EXTI9));
        __foreach_pin_inner!((PA10,GPIOA,0,10, EXTI10));
        __foreach_pin_inner!((PA11,GPIOA,0,11, EXTI11));
        __foreach_pin_inner!((PA12,GPIOA,0,12, EXTI12));
        __foreach_pin_inner!((PA13,GPIOA,0,13, EXTI13));
        __foreach_pin_inner!((PA14,GPIOA,0,14, EXTI14));
        __foreach_pin_inner!((PA15,GPIOA,0,15, EXTI15));
        __foreach_pin_inner!((PA16,GPIOA,0,16, EXTI16));
        __foreach_pin_inner!((PA17,GPIOA,0,17, EXTI17));
        __foreach_pin_inner!((PA18,GPIOA,0,18, EXTI18));
        __foreach_pin_inner!((PA19,GPIOA,0,19, EXTI19));
        __foreach_pin_inner!((PA20,GPIOA,0,20, EXTI20));
        __foreach_pin_inner!((PA21,GPIOA,0,21, EXTI21));
        __foreach_pin_inner!((PA22,GPIOA,0,22, EXTI22));
        __foreach_pin_inner!((PA23,GPIOA,0,23, EXTI23));

        __foreach_pin_inner!((PB0,GPIOB,1,0, EXTI0));
        __foreach_pin_inner!((PB1,GPIOB,1,1, EXTI1));
        __foreach_pin_inner!((PB2,GPIOB,1,2, EXTI2));
        __foreach_pin_inner!((PB3,GPIOB,1,3, EXTI3));
        __foreach_pin_inner!((PB4,GPIOB,1,4, EXTI4));
        __foreach_pin_inner!((PB5,GPIOB,1,5, EXTI5));
        __foreach_pin_inner!((PB6,GPIOB,1,6, EXTI6));
        __foreach_pin_inner!((PB7,GPIOB,1,7, EXTI7));
        __foreach_pin_inner!((PB8,GPIOB,1,8, EXTI8));
        __foreach_pin_inner!((PB9,GPIOB,1,9, EXTI9));
        __foreach_pin_inner!((PB10,GPIOB,1,10, EXTI10));
        __foreach_pin_inner!((PB11,GPIOB,1,11, EXTI11));
        __foreach_pin_inner!((PB12,GPIOB,1,12, EXTI12));
        __foreach_pin_inner!((PB13,GPIOB,1,13, EXTI13));
        __foreach_pin_inner!((PB14,GPIOB,1,14, EXTI14));
        __foreach_pin_inner!((PB15,GPIOB,1,15, EXTI15));
        __foreach_pin_inner!((PB16,GPIOB,1,16, EXTI16));
        __foreach_pin_inner!((PB17,GPIOB,1,17, EXTI17));
        __foreach_pin_inner!((PB18,GPIOB,1,18, EXTI18));
        __foreach_pin_inner!((PB19,GPIOB,1,19, EXTI19));
        __foreach_pin_inner!((PB20,GPIOB,1,20, EXTI20));
        __foreach_pin_inner!((PB21,GPIOB,1,21, EXTI21));

        __foreach_pin_inner!((PC0,GPIOC,2,0, EXTI0));
        __foreach_pin_inner!((PC1,GPIOC,2,1, EXTI1));
        __foreach_pin_inner!((PC2,GPIOC,2,2, EXTI2));
        __foreach_pin_inner!((PC3,GPIOC,2,3, EXTI3));
        __foreach_pin_inner!((PC4,GPIOC,2,4, EXTI4));
        __foreach_pin_inner!((PC5,GPIOC,2,5, EXTI5));
        __foreach_pin_inner!((PC6,GPIOC,2,6, EXTI6));
        __foreach_pin_inner!((PC7,GPIOC,2,7, EXTI7));
        __foreach_pin_inner!((PC10,GPIOC,2,10, EXTI10));
        __foreach_pin_inner!((PC11,GPIOC,2,11, EXTI11));
        __foreach_pin_inner!((PC14,GPIOC,2,14, EXTI14));
        __foreach_pin_inner!((PC15,GPIOC,2,15, EXTI15));
        __foreach_pin_inner!((PC16,GPIOC,2,16, EXTI16));
        __foreach_pin_inner!((PC17,GPIOC,2,17, EXTI17));
        __foreach_pin_inner!((PC18,GPIOC,2,18, EXTI18));
        __foreach_pin_inner!((PC19,GPIOC,2,19, EXTI19));


    };
}
foreach_pin!(
    ($pin_name:ident, $port_name:ident, $port_num:expr, $pin_num:expr, $exti_ch:ident) => {
        impl Pin for peripherals::$pin_name {
            // #[cfg(feature = "exti")]
            type ExtiChannel = peripherals::$exti_ch;
        }

        impl sealed::Pin for peripherals::$pin_name {
            #[inline]
            fn pin_port(&self) -> u8 {
                $port_num * 32 + $pin_num
            }
        }

        impl From<peripherals::$pin_name> for AnyPin {
            fn from(x: peripherals::$pin_name) -> Self {
                x.degrade()
            }
        }
    };
);

/// Enable the GPIO peripheral clock.
pub(crate) unsafe fn init() {
    let rcc = &crate::pac::RCC;

    rcc.apb2pcenr().modify(|w| {
        w.set_afioen(true);
        w.set_iopaen(true);
        w.set_iopben(true);
        w.set_iopcen(true);
    });
}

/// Disable RVSWD, use pins as GPIO
#[inline]
pub(crate) fn disable_software_debug_pins() {
    let afio = &crate::pac::AFIO;
    afio.pcfr1().modify(|w| w.set_sw_cfg(0b100));
}

impl<'d, T: Pin> embedded_hal::digital::ErrorType for Input<'d, T> {
    type Error = Infallible;
}

impl<'d, T: Pin> embedded_hal::digital::InputPin for Input<'d, T> {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<'d, T: Pin> embedded_hal::digital::ErrorType for Output<'d, T> {
    type Error = Infallible;
}

impl<'d, T: Pin> embedded_hal::digital::OutputPin for Output<'d, T> {
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_high())
    }

    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_low())
    }
}

impl<'d, T: Pin> embedded_hal::digital::StatefulOutputPin for Output<'d, T> {
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    /// Is the output pin set as low?
    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}

impl<'d, T: Pin> embedded_hal::digital::InputPin for Flex<'d, T> {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<'d, T: Pin> embedded_hal::digital::OutputPin for Flex<'d, T> {
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_high())
    }

    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_low())
    }
}

impl<'d, T: Pin> embedded_hal::digital::ErrorType for Flex<'d, T> {
    type Error = Infallible;
}

impl<'d, T: Pin> embedded_hal::digital::StatefulOutputPin for Flex<'d, T> {
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    /// Is the output pin set as low?
    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}
