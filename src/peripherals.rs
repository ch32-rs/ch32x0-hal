use crate::pac;

// We need to export this in the hal for the drivers to use

crate::peripherals! {
    SYSTICK <= SYSTICK,

    ADC <= ADC,
    I2C1 <= I2C1,
    OPA <= OPA,
    SPI1 <= SPI1,

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


    PC0 <= virtual,
    PC1 <= virtual,
    PC2 <= virtual,
    PC3 <= virtual,
    PC4 <= virtual,
    PC5 <= virtual,
    PC6 <= virtual,
    PC7 <= virtual,
    PC10 <= virtual,
    PC11 <= virtual,
    PC14 <= virtual, // CC1
    PC15 <= virtual, // CC2
    PC16 <= virtual,
    PC17 <= virtual,
    PC18 <= virtual, // DIO
    PC19 <= virtual, // DCK

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
    PA21 <= virtual,
    PA22 <= virtual,
    PA23 <= virtual,

    PB0 <= virtual,
    PB1 <= virtual,
    PB2 <= virtual,
    PB3 <= virtual,
    PB4 <= virtual,
    PB5 <= virtual,
    PB6 <= virtual,
    PB7 <= virtual,
    PB8 <= virtual,
    PB9 <= virtual,
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
}