/* CH32X035 */
MEMORY
{
	FLASH : ORIGIN = 0x00000000, LENGTH = 62k
	/* When use PIOC
	/* RAM : ORIGIN = 0x20000000, LENGTH = 16k */
	RAM : ORIGIN = 0x20000000, LENGTH = 20k
}

REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_RODATA", FLASH);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", RAM);

/* fault handlers */

PROVIDE(InstructionMisaligned = ExceptionHandler);
PROVIDE(InstructionFault = ExceptionHandler);
PROVIDE(IllegalInstruction = ExceptionHandler);
PROVIDE(Breakpoint = ExceptionHandler);
PROVIDE(LoadMisaligned = ExceptionHandler);
PROVIDE(LoadFault = ExceptionHandler);
PROVIDE(StoreMisaligned = ExceptionHandler);
PROVIDE(StoreFault = ExceptionHandler);;
PROVIDE(UserEnvCall = ExceptionHandler);
PROVIDE(SupervisorEnvCall = ExceptionHandler);
PROVIDE(MachineEnvCall = ExceptionHandler);
PROVIDE(InstructionPageFault = ExceptionHandler);
PROVIDE(LoadPageFault = ExceptionHandler);
PROVIDE(StorePageFault = ExceptionHandler);

/* core interrupt handlers */

PROVIDE(NonMaskableInt = DefaultHandler);
PROVIDE(Software = DefaultHandler);

/* external interrupt handlers */

PROVIDE(WWDG = DefaultHandler);
PROVIDE(PVD = DefaultHandler);
PROVIDE(FLASH = DefaultHandler);

PROVIDE(EXTI7_0 = DefaultHandler);
PROVIDE(AWU = DefaultHandler);
PROVIDE(DMA_CHANNEL1 = DefaultHandler);
PROVIDE(DMA_CHANNEL2 = DefaultHandler);
PROVIDE(DMA_CHANNEL3 = DefaultHandler);
PROVIDE(DMA_CHANNEL4 = DefaultHandler);
PROVIDE(DMA_CHANNEL5 = DefaultHandler);
PROVIDE(DMA_CHANNEL6 = DefaultHandler);
PROVIDE(DMA_CHANNEL7 = DefaultHandler);
PROVIDE(ADC1 = DefaultHandler);
PROVIDE(I2C1_EV = DefaultHandler);
PROVIDE(I2C1_ER = DefaultHandler);
PROVIDE(USART1 = DefaultHandler);
PROVIDE(SPI1 = DefaultHandler);
PROVIDE(TIM1_BRK = DefaultHandler);

PROVIDE(TIM1_TRG_COM = DefaultHandler);
PROVIDE(TIM1_CC = DefaultHandler);
PROVIDE(TIM2_UP_ = DefaultHandler);
PROVIDE(USART2 = DefaultHandler);
PROVIDE(EXTI15_8 = DefaultHandler);
PROVIDE(EXTI25_16 = DefaultHandler);
PROVIDE(USART3 = DefaultHandler);
PROVIDE(UART4 = DefaultHandler);
PROVIDE(DMA_CHANNEL8 = DefaultHandler);
PROVIDE(USBFS = DefaultHandler);
PROVIDE(USBFS_WKUP = DefaultHandler);
PROVIDE(PIOC = DefaultHandler);
PROVIDE(OPA = DefaultHandler);
PROVIDE(USBPD = DefaultHandler);
PROVIDE(USBPD_WKUP = DefaultHandler);
PROVIDE(TIM2_CC = DefaultHandler);
PROVIDE(TIM2_TRG_COM = DefaultHandler);
PROVIDE(TIM2_BRK = DefaultHandler);
PROVIDE(TIM3 = DefaultHandler);
