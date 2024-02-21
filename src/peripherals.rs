use crate::pac;

// We need to export this in the hal for the drivers to use

crate::peripherals! {
    SYSTICK <= SYSTICK,

    ADC <= ADC,
    I2C1 <= I2C1,
    SPI1 <= SPI1,

    OPA1 <= virtual,
    OPA2 <= virtual,

    PIOC <= virtual,

    TIM1 <= TIM1,
    TIM3 <= TIM3,

    USART1 <= USART1,
    USART2 <= USART2,
    USART3 <= USART3,
    USART4 <= USART4,

    EXTI <= EXTI,

    USBFS <= USBFS,
    USBPD <= USBPD,

    EXTI0 <= virtual,
    EXTI1 <= virtual,
    EXTI2 <= virtual,
    EXTI3 <= virtual,
    EXTI4 <= virtual,
    EXTI5 <= virtual,
    EXTI6 <= virtual,
    EXTI7 <= virtual,
    EXTI8 <= virtual,
    EXTI9 <= virtual,
    EXTI10 <= virtual,
    EXTI11 <= virtual,
    EXTI12 <= virtual,
    EXTI13 <= virtual,
    EXTI14 <= virtual,
    EXTI15 <= virtual,
    EXTI16 <= virtual,
    EXTI17 <= virtual,
    EXTI18 <= virtual,
    EXTI19 <= virtual,
    EXTI20 <= virtual,
    EXTI21 <= virtual,
    EXTI22 <= virtual,
    EXTI23 <= virtual,

    PA0 <= virtual,
    PA1 <= virtual,
    PA2 <= virtual,
    PA3 <= virtual,
    PA4 <= virtual,
    PA5 <= virtual,
    PA6 <= virtual,
    PA7 <= virtual,
    PA8 <= virtual,
    PA9 <= virtual,
    PA10 <= virtual,
    PA11 <= virtual,
    PA12 <= virtual,
    PA13 <= virtual,
    PA14 <= virtual,
    PA15 <= virtual,
    PA16 <= virtual,
    PA17 <= virtual,
    PA18 <= virtual,
    PA19 <= virtual,
    PA20 <= virtual,
    PA21 <= virtual, // RST
    PA22 <= virtual,
    PA23 <= virtual,

    PB0 <= virtual,
    PB1 <= virtual,
    PB2 <= virtual,
    PB3 <= virtual,
    PB4 <= virtual,
    PB5 <= virtual,
    PB6 <= virtual,
    PB7 <= virtual, // RST for TSSOP20 CH32X033
    PB8 <= virtual,
    PB9 <= virtual, // MCO
    PB10 <= virtual,
    PB11 <= virtual,
    PB12 <= virtual,
    PB13 <= virtual,
    PB14 <= virtual,
    PB15 <= virtual,
    PB16 <= virtual,
    PB17 <= virtual,
    PB18 <= virtual,
    PB19 <= virtual,
    PB20 <= virtual,
    PB21 <= virtual,

    PC0 <= virtual,
    PC1 <= virtual,
    PC2 <= virtual,
    PC3 <= virtual, // RST for QSOP28/TSSOP20 CH32X035
    PC4 <= virtual,
    PC5 <= virtual,
    PC6 <= virtual,
    PC7 <= virtual,
    PC10 <= virtual,
    PC11 <= virtual,
    PC14 <= virtual, // CC1
    PC15 <= virtual, // CC2
    PC16 <= virtual, // USB_DM
    PC17 <= virtual, // USB_DP
    PC18 <= virtual, // DIO, PIOC_IO0
    PC19 <= virtual, // DCK, PIOC_IO1
}
