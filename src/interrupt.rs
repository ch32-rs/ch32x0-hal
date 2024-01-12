pub use qingke::interrupt::Priority;

use crate::pac::interrupt::Interrupt as InterruptEnum;
use crate::pac::__EXTERNAL_INTERRUPTS as _;

mod sealed {
    pub trait Interrupt {}
}

/// Type-level interrupt.
///
/// This trait is implemented for all typelevel interrupt types in this module.
pub trait Interrupt: sealed::Interrupt {
    /// Interrupt enum variant.
    ///
    /// This allows going from typelevel interrupts (one type per interrupt) to
    /// non-typelevel interrupts (a single `Interrupt` enum type, with one variant per interrupt).
    const IRQ: InterruptEnum;
}

macro_rules! impl_irqs {
    ($($irqs:ident),* $(,)?) => {
        $(
            #[allow(non_camel_case_types)]
            #[doc=stringify!($irqs)]
            #[doc=" typelevel interrupt."]
            pub enum $irqs {}
            impl sealed::Interrupt for $irqs{}
            impl Interrupt for $irqs {
                const IRQ: InterruptEnum = InterruptEnum::$irqs;
            }
        )*
    }
}

impl_irqs!(USART1, USART2, USART3, USART4);
