//! rt for CH32X035

use core::arch::global_asm;

use qingke::riscv::register::mcause;

#[export_name = "error: riscv-rt appears more than once in the dependency graph"]
#[doc(hidden)]
pub static __ONCE__: () = ();

#[doc(hidden)]
pub union Vector {
    handler: unsafe extern "C" fn(),
    reserved: usize,
}

#[doc(hidden)]
#[no_mangle]
#[allow(unused_variables, non_snake_case)]
pub fn DefaultInterruptHandler() {
    loop {
        // Prevent this from turning into a UDF instruction
        // see rust-lang/rust#28728 for details
        continue;
    }
}



#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u16)]
#[allow(non_camel_case_types)]
pub enum Interrupt {
    NonMaskableInt = 2,
    HardFault = 3,
    EcallM = 5,
    EcallU = 8,
    BreakPoint = 9,
    SysTick = 12,
    Software = 14,

    // External interrupts
    ///16 - Window Watchdog interrupt
    WWDG = 16,
    ///17 - PVD through EXTI line detection interrupt
    PVD = 17,
    ///18 - Flash global interrupt
    FLASH = 18,
    ///20 - EXTI Line\[7:0\]
    ///interrupt
    EXTI7_0 = 20,
    ///21 - AWU global interrupt
    AWU = 21,
    ///22 - DMA Channel1 global interrupt
    DMA_CHANNEL1 = 22,
    ///23 - DMA Channel2 global interrupt
    DMA_CHANNEL2 = 23,
    ///24 - DMA Channel3 global interrupt
    DMA_CHANNEL3 = 24,
    ///25 - DMA Channel4 global interrupt
    DMA_CHANNEL4 = 25,
    ///26 - DMA Channel5 global interrupt
    DMA_CHANNEL5 = 26,
    ///27 - DMA Channel6 global interrupt
    DMA_CHANNEL6 = 27,
    ///28 - DMA Channel7 global interrupt
    DMA_CHANNEL7 = 28,
    ///29 - ADC global interrupt
    ADC1 = 29,
    ///30 - I2C1 event interrupt
    I2C1_EV = 30,
    ///31 - I2C1 error interrupt
    I2C1_ER = 31,
    ///32 - USART1 global interrupt
    USART1 = 32,
    ///33 - SPI1 global interrupt
    SPI1 = 33,
    ///34 - TIM1 Break interrupt
    TIM1_BRK = 34,
    ///35 - TIM1 Update interrupt
    TIM1_UP = 35,
    ///36 - TIM1 Trigger and Commutation interrupts
    TIM1_TRG = 36,
    ///37 - TIM1 Capture Compare interrupt
    TIM1_CC = 37,
    ///38 - TIM2 Update interrupt
    TIM2_UP = 38,
    ///39 - USART2 global interrupt
    USART2 = 39,
    ///40 - EXTI Line\[15:8\] interrupts
    EXTI15_8 = 40,
    ///41 - EXTI Line\[25:16\] interrupts
    EXTI25_16 = 41,
    ///42 - USART3 global interrupt
    USART3 = 42,
    ///43 - UART4 global interrupt
    UART4 = 43,
    ///44 - DMA Channel8 global interrupt
    DMA_CHANNEL8 = 44,
    ///45 - USBFS
    USBFS = 45,
    ///46 - USBFS_WKUP
    USBFS_WKUP = 46,
    ///49 - USBPD global interrupt
    USBPD = 49,
    ///50 - USBPD_WKUP global interrupt
    USBPD_WKUP = 50,
    ///51 - TIM2 Capture Compare interrupt
    TIM2_CC = 51,
    ///52 - TIM2 Trigger and Commutation interrupts
    TIM2_TRG = 52,
    ///53 - TIM2 Break interrupt
    TIM2_BRK = 53,
    ///54 - TIM3 global interrupt
    TIM3 = 54,
}

// Overwrites PAC's interrupt handlers
extern "C" {
    fn NonMaskableInt();

    fn HardFault();

    fn EcallM();

    fn EcallU();

    fn BreakPoint();

    fn SysTick();

    fn Software();

    // External interrupts
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
    fn TIM1_UP();
    fn TIM1_TRG();
    fn TIM1_CC();
    fn TIM2_UP();
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
    fn TIM2_CC();
    fn TIM2_TRG();
    fn TIM2_BRK();
    fn TIM3();
}

#[doc(hidden)]
#[link_section = ".trap.interrupts"]
#[no_mangle]
pub static __INTERRUPTS: [Vector; 55] = [
    Vector { reserved: 0 },
    Vector { reserved: 0 }, // reset?
    // 2: Non-Maskable Interrupt.
    Vector {
        handler: NonMaskableInt,
    },
    // 3: Hard Fault Interrupt.
    Vector { handler: HardFault },
    Vector { reserved: 0 },
    // 5: ECALL-M
    Vector { handler: EcallM },
    Vector { reserved: 0 },
    Vector { reserved: 0 },
    // 8: ECALL-U
    Vector { handler: EcallU },
    // 9: Breakpoint
    Vector {
        handler: BreakPoint,
    },
    // 10-11
    Vector { reserved: 0 },
    Vector { reserved: 0 },
    // 12
    Vector { handler: SysTick },
    Vector { reserved: 0 },
    Vector { handler: Software },
    Vector { reserved: 0 },
    // External interrupts
    // 15
    Vector { handler: WWDG },
    Vector { handler: PVD },
    Vector { handler: FLASH },
    Vector { reserved: 0 },
    Vector { handler: EXTI7_0 },
    Vector { handler: AWU },
    Vector {
        handler: DMA_CHANNEL1,
    },
    Vector {
        handler: DMA_CHANNEL2,
    },
    Vector {
        handler: DMA_CHANNEL3,
    },
    Vector {
        handler: DMA_CHANNEL4,
    },
    Vector {
        handler: DMA_CHANNEL5,
    },
    Vector {
        handler: DMA_CHANNEL6,
    },
    Vector {
        handler: DMA_CHANNEL7,
    },
    Vector { handler: ADC1 },
    Vector { handler: I2C1_EV },
    Vector { handler: I2C1_ER },
    Vector { handler: USART1 },
    Vector { handler: SPI1 },
    Vector { handler: TIM1_BRK },
    Vector { handler: TIM1_UP },
    Vector {
        handler: TIM1_TRG,
    },
    Vector { handler: TIM1_CC },
    Vector { handler: TIM2_UP },
    Vector { handler: USART2 },
    Vector { handler: EXTI15_8 },
    Vector { handler: EXTI25_16 },
    Vector { handler: USART3 },
    Vector { handler: UART4 },
    Vector {
        handler: DMA_CHANNEL8,
    },
    Vector { handler: USBFS },
    Vector {
        handler: USBFS_WKUP,
    },
    Vector { reserved: 0 },
    Vector { reserved: 0 },
    Vector { handler: USBPD },
    Vector {
        handler: USBPD_WKUP,
    },
    Vector { handler: TIM2_CC },
    Vector {
        handler: TIM2_TRG,
    },
    Vector { handler: TIM2_BRK },
    Vector { handler: TIM3 },
];


/// Trap entry point rust (_start_trap_rust)
///
/// `scause`/`mcause` is read to determine the cause of the trap. XLEN-1 bit indicates
/// if it's an interrupt or an exception. The result is examined and ExceptionHandler
/// or one of the core interrupt handlers is called.
///
/// # Safety
///
/// This function must be called only from assembly `_start_trap` function.
/// Do **NOT** call this function directly.
#[link_section = ".trap.rust"]
#[export_name = "_start_trap_rust"]
pub unsafe extern "C" fn start_trap_rust(trap_frame: *const TrapFrame) {
    extern "C" {
        fn ExceptionHandler(trap_frame: &TrapFrame);
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
    } else if code < __INTERRUPTS.len() {
        let h = &__INTERRUPTS[code];
        if let Some(handler) = h {
            handler();
        } else {
            DefaultHandler();
        }
    } else {
        DefaultHandler();
    }
}



macro_rules! cfg_global_asm {
    {@inner, [$($x:tt)*], } => {
        global_asm!{$($x)*}
    };
    (@inner, [$($x:tt)*], #[cfg($meta:meta)] $asm:literal, $($rest:tt)*) => {
        #[cfg($meta)]
        cfg_global_asm!{@inner, [$($x)* $asm,], $($rest)*}
        #[cfg(not($meta))]
        cfg_global_asm!{@inner, [$($x)*], $($rest)*}
    };
    {@inner, [$($x:tt)*], $asm:literal, $($rest:tt)*} => {
        cfg_global_asm!{@inner, [$($x)* $asm,], $($rest)*}
    };
    {$($asms:tt)*} => {
        cfg_global_asm!{@inner, [], $($asms)*}
    };
}

cfg_global_asm! {
    "
    .section    .init,\"ax\"
    .global _start
    .align  1
//    .option norvc
_start:
    j handle_reset
    ",
    "
    .section    .handle_reset,\"ax\",@progbits
    .weak   handle_reset
    .align  1
handle_reset:
    .option push
    .option norelax
    la gp, __global_pointer$
    .option pop
    la sp, _stack_top
    ",
    // load highcode from flash to ram
    "
    la a0, _highcode_lma
    la a1, _highcode_vma_start
    la a2, _highcode_vma_end
    bgeu a1, a2, 2f
1:
    lw t0, (a0)
    sw t0, (a1)
    addi a0, a0, 4
    addi a1, a1, 4
    bltu a1, a2, 1b
    ",
    // load data from flash to ram
    "
2:
    la a0, _data_lma
    la a1, _data_vma
    la a2, _edata
    bgeu a1, a2, 2f
1:
    lw t0, (a0)
    sw t0, (a1)
    addi a0, a0, 4
    addi a1, a1, 4
    bltu a1, a2, 1b
2:
    ",
    // clear bss section
    "
    la a0, _sbss
    la a1, _ebss
    bgeu a0, a1, 2f
1:
    sw zero, (a0)
    addi a0, a0, 4
    bltu a0, a1, 1b
2:
    ",
    // corecfgr: 流水线控制位 & 动态预测控制位
    // corecfgr: Pipeline control bit & Dynamic prediction control
    "
    li t0, 0x1f
    csrw 0xbc0, t0",
    // 打开嵌套中断、硬件压栈功能
    // intsyscr: Open nested interrupts and hardware stack functions
    // 0x3 both nested interrupts and hardware stack
    // 0x1 only hardware stack
    "
    li t0, 0x3
    csrw 0x804, t0",
    // Restore state
    // - use 0x88 to set mpp=0, return to user mode
    // - use 0x1888 to set mpp=3, return to machine mode
    "
    li t0, 0x88
    csrs mstatus, t0
    ",
    // ~~配置向量表模式为绝对地址模式~~
    //
    "
    la t0, __vector_base
    ori t0, t0, 3
    csrw mtvec, t0
    ",
    // return to main
    "
    la t0, main
    csrw mepc, t0

    mret
    ",
}
