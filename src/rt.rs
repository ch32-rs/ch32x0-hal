use core::arch::{asm, global_asm};

use qingke::register::gintenr;
use qingke::register::mtvec::{self, TrapMode};
use qingke::riscv::register::mcause;

extern "C" {
    fn InstructionMisaligned();
    fn InstructionFault();
    fn IllegalInstruction();
    fn Breakpoint();
    fn LoadMisaligned();
    fn LoadFault();
    fn StoreMisaligned();
    fn StoreFault();
    fn UserEnvCall();
    fn MachineEnvCall();
}

#[doc(hidden)]
#[no_mangle]
pub static __EXCEPTIONS: [Option<unsafe extern "C" fn(&TrapFrame)>; 12] = [
    Some(InstructionMisaligned),
    Some(InstructionFault),
    Some(IllegalInstruction),
    Some(Breakpoint),
    Some(LoadMisaligned),
    Some(LoadFault),
    Some(StoreMisaligned),
    Some(StoreFault),
    Some(UserEnvCall),
    None,
    None,
    Some(MachineEnvCall),
];

extern "C" {
    fn NonMaskableInt();
    fn SysTick();
    fn Software();
}

// Core interrupts
#[doc(hidden)]
#[no_mangle]
pub static __CORE_INTERRUPTS: [Option<unsafe extern "C" fn()>; 16] = [
    None,
    None,
    Some(NonMaskableInt), // 2
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    Some(SysTick), // 12
    None,
    Some(Software), // 14
    None,
];

extern "C" {
    fn SYS_TICK();
    fn WWDG();
    fn PVD();
    fn FLASH();
    fn EXTI7_0();
    fn AWU();
    fn DMA_CHANNEL1();
    fn DMA_CHANNEL2();
    fn DMA_CHANNEL3();
    fn DMA_CHANNEL4();
    fn DMA_CHANNEL5();
    fn DMA_CHANNEL6();
    fn DMA_CHANNEL7();
    fn ADC1();
    fn I2C1_EV();
    fn I2C1_ER();
    fn USART1();
    fn SPI1();
    fn TIM1_BRK();
    fn TIM1_TRG_COM();
    fn TIM1_CC();
    fn TIM2_UP_();
    fn USART2();
    fn EXTI15_8();
    fn EXTI25_16();
    fn USART3();
    fn UART4();
    fn DMA_CHANNEL8();
    fn USBFS();
    fn USBFS_WKUP();
    fn USBPD();
    fn USBPD_WKUP();
    fn PIOC();
    fn OPA();
    fn TIM2_CC();
    fn TIM2_TRG_COM();
    fn TIM2_BRK();
    fn TIM3();
}

#[doc(hidden)]
#[no_mangle]
pub static __EXTERNAL_INTERRUPTS: [Option<unsafe extern "C" fn()>; 55] = [
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None, // Some(SYS_TICK),
    None,
    None,
    None,
    Some(WWDG),
    Some(PVD),
    Some(FLASH),
    None,
    Some(EXTI7_0),
    Some(AWU),
    Some(DMA_CHANNEL1),
    Some(DMA_CHANNEL2),
    Some(DMA_CHANNEL3),
    Some(DMA_CHANNEL4),
    Some(DMA_CHANNEL5),
    Some(DMA_CHANNEL6),
    Some(DMA_CHANNEL7),
    Some(ADC1),
    Some(I2C1_EV),
    Some(I2C1_ER),
    Some(USART1),
    Some(SPI1),
    Some(TIM1_BRK),
    None,
    Some(TIM1_TRG_COM),
    Some(TIM1_CC),
    Some(TIM2_UP_),
    Some(USART2),
    Some(EXTI15_8),
    Some(EXTI25_16),
    Some(USART3),
    Some(UART4),
    Some(DMA_CHANNEL8),
    Some(USBFS),
    Some(USBFS_WKUP),
    Some(PIOC),
    Some(OPA),
    Some(USBPD),
    Some(USBPD_WKUP),
    Some(TIM2_CC),
    Some(TIM2_TRG_COM),
    Some(TIM2_BRK),
    Some(TIM3),
];

#[link_section = ".trap.rust"]
#[export_name = "_ch32x0_star_trap_rust"]
pub unsafe extern "C" fn start_trap_rust(trap_frame: *const TrapFrame) {
    extern "C" {
        fn ExceptionHandler();
        fn DefaultHandler();
    }

    let cause = mcause::read();
    let code = cause.code();

    if cause.is_exception() {
        let trap_frame = &*trap_frame;
        if code < __EXCEPTIONS.len() {
            let h = &__EXCEPTIONS[code];
            if let Some(handler) = h {
                handler(trap_frame);
            } else {
                ExceptionHandler(trap_frame);
            }
        } else {
            ExceptionHandler(trap_frame);
        }
        ExceptionHandler(trap_frame)
    } else if code < __CORE_INTERRUPTS.len() {
        let h = &__CORE_INTERRUPTS[code];
        if let Some(handler) = h {
            handler();
        } else {
            DefaultHandler();
        }
    } else if code < __EXTERNAL_INTERRUPTS.len() {
        let h = &__EXTERNAL_INTERRUPTS[code];
        if let Some(handler) = h {
            handler();
        } else {
            DefaultHandler();
        }
    } else {
        DefaultHandler();
    }
}

// override _start_trap in riscv-rt
global_asm!(
    r#"
        .section .trap, "ax"
        .global _start_trap
    _start_trap:
        addi sp, sp, -4
        sw ra, 0(sp)
        jal _ch32x0_star_trap_rust
        lw ra, 0(sp)
        addi sp, sp, 4
        mret
    "#
);

#[no_mangle]
#[export_name = "_setup_interrupts"]
unsafe fn ch32x0_setup_interrupts() {
    extern "C" {
        fn _start_trap();
    }

    crate::debug::SDIPrint::enable();
    // enable hardware stack push
    asm!(
        "
        li t0, 0x1f
        csrw 0xbc0, t0
        li t0, 0x3
        csrw 0x804, t0
        "
    );
    mtvec::write(_start_trap as usize, TrapMode::Direct);
    gintenr::set_enable();
}
