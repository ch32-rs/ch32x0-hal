//! 4 USARTs
//!
//! - No split of BasicInstance and FullInstance

/*
Full-duplex or half-duplex synchronous or asynchronous communication
NRZ data format
Fractional baud rate generator, up to 3Mbps
Programmable data length
Configurable stop bits
Supports LIN, IrDA encoder, smart card
Supports DMA
Multiple interrupt sources
 */

use core::marker::PhantomData;

use crate::gpio::Pull;
use crate::{into_ref, pac, peripherals, Peripheral};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataBits {
    DataBits8 = 0,
    DataBits9 = 1,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Parity {
    ParityNone,
    ParityEven,
    ParityOdd,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StopBits {
    #[doc = "1 stop bit"]
    STOP1 = 0,
    #[doc = "0.5 stop bits"]
    STOP0P5 = 0b01,
    #[doc = "2 stop bits"]
    STOP2 = 0b10,
    #[doc = "1.5 stop bits"]
    STOP1P5 = 0b11,
}

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Config {
    pub baudrate: u32,
    pub data_bits: DataBits,
    pub stop_bits: StopBits,
    pub parity: Parity,
}
impl Default for Config {
    /// 115200 8N1
    fn default() -> Self {
        Self {
            baudrate: 115200,
            data_bits: DataBits::DataBits8,
            stop_bits: StopBits::STOP1,
            parity: Parity::ParityNone,
        }
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Framing error
    // FE
    Framing,
    /// Noise error
    // NE
    Noise,
    /// RX buffer overrun
    // ORE
    Overrun,
    /// Parity check error
    // PE
    Parity,
    /// Buffer too large for DMA
    BufferTooLong,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    BaudrateTooLow,
    BaudrateTooHigh,
}

pub struct UartTx<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
}

impl<'d, T: Instance> UartTx<'d, T> {
    pub fn new<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,

        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        T::enable_and_reset();

        Self::new_inner(peri, tx, config)
    }

    fn new_inner<const REMAP: u8>(
        _peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(_peri, tx);

        tx.set_as_af_output();
        T::set_remap(REMAP);

        let rb = T::regs();
        configure(rb, &config, true, false)?;

        // TODO: async state

        Ok(Self { phantom: PhantomData })
    }

    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        let rb = T::regs();

        for &c in buffer {
            while rb.statr().read().tc().bit_is_clear() {} // wait tx complete
            rb.datar().write(|w| unsafe { w.dr().bits(c as u16) });
        }
        Ok(())
    }

    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        let rb = T::regs();

        while rb.statr().read().txe().bit_is_clear() {} // wait txe
        Ok(())
    }
}

pub struct UartRx<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
}

impl<'d, T: Instance> UartRx<'d, T> {
    pub fn new<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,

        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        T::enable_and_reset();

        Self::new_inner(peri, rx, config)
    }

    fn new_inner<const REMAP: u8>(
        _peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(_peri, rx);

        rx.set_as_input(Pull::None);
        T::set_remap(REMAP);

        let rb = T::regs();
        configure(rb, &config, false, true)?;

        Ok(Self { phantom: PhantomData })
    }

    /// Read a single u8 if there is one avaliable
    pub fn nb_read(&mut self) -> Result<u8, nb::Error<Error>> {
        let rb = T::regs();

        // while rb.statr().read().rxne().bit_is_clear() {} // wait rxne
        if rb.statr().read().rxne().bit_is_clear() {
            Err(nb::Error::WouldBlock)
        } else {
            Ok(rb.datar().read().dr().bits() as u8) // FIXME: how to handle 9 bits
        }
    }

    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        let rb = T::regs();

        for c in buffer {
            while rb.statr().read().rxne().bit_is_clear() {} // wait rxne
            *c = rb.datar().read().dr().bits() as u8; // FIXME: how to handle 9 bits
        }
        Ok(())
    }
}

/// Bidirectional UART Driver
pub struct Uart<'d, T: Instance> {
    tx: UartTx<'d, T>,
    rx: UartRx<'d, T>,
}

impl<'d, T: Instance> Uart<'d, T> {
    pub fn new<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        T::enable_and_reset();

        Self::new_inner(peri, tx, rx, config)
    }

    /// Half-duplex
    ///
    /// Note: Half duplex requires TX pin to have a pull-up resistor
    pub fn new_half_duplex_on_tx<const REMAP: u8>(
        _peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        T::enable_and_reset();

        into_ref!(_peri, tx);

        // 推挽复用输出(外加上拉)
        tx.set_as_af_output();
        T::set_remap(REMAP);

        let rb = T::regs();
        configure(rb, &config, true, true)?;

        rb.ctlr3().modify(|_, w| w.hdsel().set_bit()); // half duplex

        rb.statr().modify(|_, w| w.rxne().clear_bit()); // clear rxne
        let _ = rb.datar().read().dr(); // clear rxne

        Ok(Self {
            tx: UartTx { phantom: PhantomData },
            rx: UartRx { phantom: PhantomData },
        })
    }

    fn new_inner<const REMAP: u8>(
        _peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(_peri, tx, rx);

        tx.set_as_af_output();
        rx.set_as_input(Pull::None);
        T::set_remap(REMAP);

        let rb = T::regs();
        configure(rb, &config, true, true)?;

        Ok(Self {
            tx: UartTx { phantom: PhantomData },
            rx: UartRx { phantom: PhantomData },
        })
    }

    pub fn split(self) -> (UartTx<'d, T>, UartRx<'d, T>) {
        (self.tx, self.rx)
    }

    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        let rb = T::regs();

        for &c in buffer {
            while rb.statr().read().txe().bit_is_clear() {} // wait tx complete
            rb.datar().write(|w| unsafe { w.dr().bits(c as u16) });
        }
        Ok(())
    }

    /// Read a single u8 if there is one avaliable
    pub fn nb_read(&mut self) -> Result<u8, nb::Error<Error>> {
        self.rx.nb_read()
    }

    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        self.rx.blocking_read(buffer)
    }
}

pub(crate) mod sealed {
    // use embassy_sync::waitqueue::AtomicWaker;

    use super::*;
    use crate::interrupt;

    pub trait Instance {
        type Interrupt: interrupt::Interrupt;

        fn regs() -> &'static pac::usart1::RegisterBlock;

        fn enable_and_reset();
        // fn state() -> &'static ();

        // remap for USART1 to USART4
        fn set_remap(remap: u8);
    }
}

fn configure(
    rb: &pac::usart1::RegisterBlock,
    config: &Config,
    enable_tx: bool,
    enable_rx: bool,
) -> Result<(), ConfigError> {
    if !enable_rx && !enable_tx {
        panic!("USART: At least one of RX or TX should be enabled");
    }

    rb.ctlr2().modify(|_, w| w.stop().variant(config.stop_bits as u8));

    rb.ctlr1().modify(|_, w| {
        w.m()
            .variant(config.data_bits as u8 != 0)
            .pce()
            .variant(config.parity != Parity::ParityNone)
            .ps()
            .variant(config.parity == Parity::ParityOdd) // 1 for odd parity, 0 for even parity
            .te()
            .bit(enable_tx)
            .re()
            .bit(enable_rx)
    });

    // HCLK/(16*USARTDIV)
    // USARTDIV = DIV_M+(DIV_F/16)  via USART_BRR

    let apbclock = crate::rcc::clocks().hclk.to_Hz();

    let div_m = 25 * apbclock / (4 * config.baudrate);
    let mut tmpreg = (div_m / 100) << 4;

    let div_f = div_m - 100 * (tmpreg >> 4);
    tmpreg |= ((div_f * 16 + 50) / 100) & 0x0F;

    rb.brr().write(|w| unsafe { w.bits(tmpreg) });

    // enable uart
    rb.ctlr1().modify(|_, w| w.ue().set_bit());

    Ok(())
}

// embedded-hal
mod eh1 {
    use super::*;
}

// Peripheral traits

pub trait Instance: Peripheral<P = Self> + sealed::Instance + 'static + Send {}

macro_rules! impl_uart {
    ($inst:ident, $irq:ident, $rst_reg:ident, $rst_field:ident, $en_reg:ident, $en_field:ident, $remap_field:ident) => {
        impl sealed::Instance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::$irq;

            fn regs() -> &'static crate::pac::usart1::RegisterBlock {
                unsafe { &*crate::pac::$inst::PTR }
            }

            fn enable_and_reset() {
                let rcc = unsafe { &*crate::pac::RCC::PTR };

                rcc.$rst_reg().modify(|_, w| w.$rst_field().set_bit());
                rcc.$rst_reg().modify(|_, w| w.$rst_field().clear_bit());
                rcc.$en_reg().modify(|_, w| w.$en_field().set_bit());
            }

            fn set_remap(remap: u8) {
                let afio = unsafe { &*pac::AFIO::PTR };
                afio.pcfr1().modify(|_, w| w.$remap_field().variant(remap));
            }
        }

        impl Instance for peripherals::$inst {}
    };
}

impl_uart!(USART1, USART1, apb2prstr, usart1rst, apb2pcenr, usart1en, usart1rm);
impl_uart!(USART2, USART2, apb1prstr, usart2rst, apb1pcenr, usart2en, usart2rm);
impl_uart!(USART3, USART3, apb1prstr, usart3rst, apb1pcenr, usart3en, usart3rm);
impl_uart!(USART4, USART4, apb1prstr, usart4rst, apb1pcenr, uart4en, usart4rm); // TODO: rename in PAC

macro_rules! pin_trait {
    ($signal:ident, $instance:path) => {
        pub trait $signal<T: $instance, const REMAP: u8>: crate::gpio::Pin {}
    };
}

pin_trait!(RxPin, Instance);
pin_trait!(TxPin, Instance);
pin_trait!(CtsPin, Instance);
pin_trait!(RtsPin, Instance);
pin_trait!(CkPin, Instance);

macro_rules! pin_trait_impl {
    (crate::$mod:ident::$trait:ident, $instance:ident, $pin:ident, $remap:expr) => {
        impl crate::$mod::$trait<crate::peripherals::$instance, $remap> for crate::peripherals::$pin {}
    };
}

// USART1

// 0, RX/PB11，CTS/PC16，TX/PB10，CK/PB9， RTS/PC17
pin_trait_impl!(crate::usart::RxPin, USART1, PB11, 0);
pin_trait_impl!(crate::usart::CtsPin, USART1, PC16, 0);
pin_trait_impl!(crate::usart::TxPin, USART1, PB10, 0);
pin_trait_impl!(crate::usart::CkPin, USART1, PB9, 0);
pin_trait_impl!(crate::usart::RtsPin, USART1, PC17, 0);

// 01, RX/PA11，CTS/PC16，TX/PA10，CK/PB9， RTS/PC17)
pin_trait_impl!(crate::usart::RxPin, USART1, PA11, 1);
pin_trait_impl!(crate::usart::CtsPin, USART1, PC16, 1);
pin_trait_impl!(crate::usart::TxPin, USART1, PA10, 1);
pin_trait_impl!(crate::usart::CkPin, USART1, PB9, 1);
pin_trait_impl!(crate::usart::RtsPin, USART1, PC17, 1);

// 10:映射(RX/PB11，CTS/PA9，TX/PB10，CK/PB5，RTS/PA8)
pin_trait_impl!(crate::usart::RxPin, USART1, PB11, 2);
pin_trait_impl!(crate::usart::CtsPin, USART1, PA9, 2);
pin_trait_impl!(crate::usart::TxPin, USART1, PB10, 2);
pin_trait_impl!(crate::usart::CkPin, USART1, PB5, 2);
pin_trait_impl!(crate::usart::RtsPin, USART1, PA8, 2);

// 11:映射(RX/PB2，CTS/PA13，TX/PA7，CK/PB12， RTS/PA14)
pin_trait_impl!(crate::usart::RxPin, USART1, PB2, 3);
pin_trait_impl!(crate::usart::CtsPin, USART1, PA13, 3);
pin_trait_impl!(crate::usart::TxPin, USART1, PA7, 3);
pin_trait_impl!(crate::usart::CkPin, USART1, PB12, 3);
pin_trait_impl!(crate::usart::RtsPin, USART1, PA14, 3);

// USART2

// 000:映射(RX/PA3，CTS/PA0，TX/PA2，CK/PA4， RTS/PA1)
pin_trait_impl!(crate::usart::RxPin, USART2, PA3, 0);
pin_trait_impl!(crate::usart::CtsPin, USART2, PA0, 0);
pin_trait_impl!(crate::usart::TxPin, USART2, PA2, 0);
pin_trait_impl!(crate::usart::CkPin, USART2, PA4, 0);
pin_trait_impl!(crate::usart::RtsPin, USART2, PA1, 0);

// 001:映射(RX/PA19，CTS/PA1，TX/PA20，CK/PA23， RTS/PA2)
pin_trait_impl!(crate::usart::RxPin, USART2, PA19, 1);
pin_trait_impl!(crate::usart::CtsPin, USART2, PA1, 1);
pin_trait_impl!(crate::usart::TxPin, USART2, PA20, 1);
pin_trait_impl!(crate::usart::CkPin, USART2, PA23, 1);
pin_trait_impl!(crate::usart::RtsPin, USART2, PA2, 1);

// 010:映射(RX/PA16，CTS/PA17，TX/PA15， CK/PA22，RTS/PA21)
pin_trait_impl!(crate::usart::RxPin, USART2, PA16, 2);
pin_trait_impl!(crate::usart::CtsPin, USART2, PA17, 2);
pin_trait_impl!(crate::usart::TxPin, USART2, PA15, 2);
pin_trait_impl!(crate::usart::CkPin, USART2, PA22, 2);
pin_trait_impl!(crate::usart::RtsPin, USART2, PA21, 2);

// 011:映射(RX/PC1，CTS/PC2，TX/PC0，CK/PB20， RTS/PC3)
pin_trait_impl!(crate::usart::RxPin, USART2, PC1, 3);
pin_trait_impl!(crate::usart::CtsPin, USART2, PC2, 3);
pin_trait_impl!(crate::usart::TxPin, USART2, PC0, 3);
pin_trait_impl!(crate::usart::CkPin, USART2, PB20, 3);
pin_trait_impl!(crate::usart::RtsPin, USART2, PC3, 3);

// 1xx:映射(RX/PA16，CTS/PA17，TX/PA15， CK/PA22，RTS/PC3)
pin_trait_impl!(crate::usart::RxPin, USART2, PA16, 4);
pin_trait_impl!(crate::usart::CtsPin, USART2, PA17, 4);
pin_trait_impl!(crate::usart::TxPin, USART2, PA15, 4);
pin_trait_impl!(crate::usart::CkPin, USART2, PA22, 4);
pin_trait_impl!(crate::usart::RtsPin, USART2, PC3, 4);

// USART3

// 00:映射(RX/PB4，CTS/PB6，TX/PB3，CK/PB5， RTS/PB7)
pin_trait_impl!(crate::usart::RxPin, USART3, PB4, 0);
pin_trait_impl!(crate::usart::CtsPin, USART3, PB6, 0);
pin_trait_impl!(crate::usart::TxPin, USART3, PB3, 0);
pin_trait_impl!(crate::usart::CkPin, USART3, PB5, 0);
pin_trait_impl!(crate::usart::RtsPin, USART3, PB7, 0);

// 01:映射(RX/PC19，CTS/PB6，TX/PC18，CK/PB5， RTS/PB7)
pin_trait_impl!(crate::usart::RxPin, USART3, PC19, 1);
pin_trait_impl!(crate::usart::CtsPin, USART3, PB6, 1);
pin_trait_impl!(crate::usart::TxPin, USART3, PC18, 1);
pin_trait_impl!(crate::usart::CkPin, USART3, PB5, 1);
pin_trait_impl!(crate::usart::RtsPin, USART3, PB7, 1);

// 10:映射(RX/PB14，CTS/PA3，TX/PA18，CK/PB8， RTS/PA4)
pin_trait_impl!(crate::usart::RxPin, USART3, PB14, 2);
pin_trait_impl!(crate::usart::CtsPin, USART3, PA3, 2);
pin_trait_impl!(crate::usart::TxPin, USART3, PA18, 2);
pin_trait_impl!(crate::usart::CkPin, USART3, PB8, 2);
pin_trait_impl!(crate::usart::RtsPin, USART3, PA4, 2);

// USART4

// 000:映射(RX/PB1，CTS/PB15，TX/PB0，CK/PB2， RTS/PA8)
pin_trait_impl!(crate::usart::RxPin, USART4, PB1, 0);
pin_trait_impl!(crate::usart::CtsPin, USART4, PB15, 0);
pin_trait_impl!(crate::usart::TxPin, USART4, PB0, 0);
pin_trait_impl!(crate::usart::CkPin, USART4, PB2, 0);
pin_trait_impl!(crate::usart::RtsPin, USART4, PA8, 0);

// 001:映射(RX/PA9，CTS/PA7，TX/PA5，CK/PA6， RTS/PB21)
pin_trait_impl!(crate::usart::RxPin, USART4, PA9, 1);
pin_trait_impl!(crate::usart::CtsPin, USART4, PA7, 1);
pin_trait_impl!(crate::usart::TxPin, USART4, PA5, 1);
pin_trait_impl!(crate::usart::CkPin, USART4, PA6, 1);
pin_trait_impl!(crate::usart::RtsPin, USART4, PB21, 1);

// 010:映射(RX/PC17，CTS/PB15，TX/PC16，CK/PB2， 0 RTS/PA8)
pin_trait_impl!(crate::usart::RxPin, USART4, PC17, 2);
pin_trait_impl!(crate::usart::CtsPin, USART4, PB15, 2);
pin_trait_impl!(crate::usart::TxPin, USART4, PC16, 2);
pin_trait_impl!(crate::usart::CkPin, USART4, PB2, 2);
pin_trait_impl!(crate::usart::RtsPin, USART4, PA8, 2);

// 011:映射(RX/PA10，CTS/PA14，TX/PB9，CK/PB8， RTS/PA13)
pin_trait_impl!(crate::usart::RxPin, USART4, PA10, 3);
pin_trait_impl!(crate::usart::CtsPin, USART4, PA14, 3);
pin_trait_impl!(crate::usart::TxPin, USART4, PB9, 3);
pin_trait_impl!(crate::usart::CkPin, USART4, PB8, 3);
pin_trait_impl!(crate::usart::RtsPin, USART4, PA13, 3);

// 1x0:映射(RX/PC19，CTS/PA5，TX/PB13，CK/PA8， RTS/PA6)
pin_trait_impl!(crate::usart::RxPin, USART4, PC19, 4);
pin_trait_impl!(crate::usart::CtsPin, USART4, PA5, 4);
pin_trait_impl!(crate::usart::TxPin, USART4, PB13, 4);
pin_trait_impl!(crate::usart::CkPin, USART4, PA8, 4);
pin_trait_impl!(crate::usart::RtsPin, USART4, PA6, 4);

// 1x1:映射(RX/PC16，CTS/PB15，TX/PC17，CK/PB2， RTS/PA8)
pin_trait_impl!(crate::usart::RxPin, USART4, PC16, 5);
pin_trait_impl!(crate::usart::CtsPin, USART4, PB15, 5);
pin_trait_impl!(crate::usart::TxPin, USART4, PC17, 5);
pin_trait_impl!(crate::usart::CkPin, USART4, PB2, 5);
pin_trait_impl!(crate::usart::RtsPin, USART4, PA8, 5);
