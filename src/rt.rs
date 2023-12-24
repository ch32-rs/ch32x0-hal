use core::arch::{asm, global_asm};

use crate::pac::__EXTERNAL_INTERRUPTS;
use qingke::{
    register::{
        gintenr,
        mtvec::{self, TrapMode},
    },
    riscv::register::mcause,
};
use riscv_rt::TrapFrame;

extern "C" {
    fn InstructionMisaligned(trap_frame: &TrapFrame);
    fn InstructionFault(trap_frame: &TrapFrame);
    fn IllegalInstruction(trap_frame: &TrapFrame);
    fn Breakpoint(trap_frame: &TrapFrame);
    fn LoadMisaligned(trap_frame: &TrapFrame);
    fn LoadFault(trap_frame: &TrapFrame);
    fn StoreMisaligned(trap_frame: &TrapFrame);
    fn StoreFault(trap_frame: &TrapFrame);
    fn UserEnvCall(trap_frame: &TrapFrame);
    fn MachineEnvCall(trap_frame: &TrapFrame);
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

#[link_section = ".trap.rust"]
#[export_name = "_ch32x0_star_trap_rust"]
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
    } else if code < __CORE_INTERRUPTS.len() {
        let h = &__CORE_INTERRUPTS[code];
        if let Some(handler) = h {
            handler();
        } else {
            DefaultHandler();
        }
    } else if code < __EXTERNAL_INTERRUPTS.len() {
        let h = &__CORE_INTERRUPTS[code];
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
    crate::println!("begin ok");
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
