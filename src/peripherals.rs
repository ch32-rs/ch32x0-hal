use crate::pac;

// We need to export this in the hal for the drivers to use

crate::peripherals! {
    SYSTICK,
    AFIO,
    GPIOA,
    GPIOB,
    GPIOC,

    ADC1,
    I2C1,
    SPI1,

    OPA1,
    OPA2,

    PIOC,

    TIM1,
    TIM2,
    TIM3,

    USART1,
    USART2,
    USART3,
    USART4,

    EXTI,

    USBFS,
    USBPD,

    EXTI0,
    EXTI1,
    EXTI2,
    EXTI3,
    EXTI4,
    EXTI5,
    EXTI6,
    EXTI7,
    EXTI8,
    EXTI9,
    EXTI10,
    EXTI11,
    EXTI12,
    EXTI13,
    EXTI14,
    EXTI15,
    EXTI16,
    EXTI17,
    EXTI18,
    EXTI19,
    EXTI20,
    EXTI21,
    EXTI22,
    EXTI23,

    PA0,
    PA1,
    PA2,
    PA3,
    PA4,
    PA5,
    PA6,
    PA7,
    PA8,
    PA9,
    PA10,
    PA11,
    PA12,
    PA13,
    PA14,
    PA15,
    PA16,
    PA17,
    PA18,
    PA19,
    PA20,
    PA21, // RST
    PA22,
    PA23,

    PB0,
    PB1,
    PB2,
    PB3,
    PB4,
    PB5,
    PB6,
    PB7, // RST for TSSOP20 CH32X033
    PB8,
    PB9, // MCO
    PB10,
    PB11,
    PB12,
    PB13,
    PB14,
    PB15,
    PB16,
    PB17,
    PB18,
    PB19,
    PB20,
    PB21,

    PC0,
    PC1,
    PC2,
    PC3, // RST for QSOP28/TSSOP20 CH32X035
    PC4,
    PC5,
    PC6,
    PC7,
    PC10,
    PC11,
    PC14, // CC1
    PC15, // CC2
    PC16, // USB_DM
    PC17, // USB_DP
    PC18, // DIO, PIOC_IO0
    PC19, // DCK, PIOC_IO1
}
