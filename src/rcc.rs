use fugit::HertzU32 as Hertz;

use crate::pac;

const HSI_FREQUENCY: Hertz = Hertz::from_raw(48_000_000);

// Power on default: HPRE = 0b0101 = Div6
const DEFAULT_FREQUENCY: Hertz = Hertz::from_raw(8_000_000);

static mut CLOCKS: Clocks = Clocks {
    // Power on default
    sysclk: DEFAULT_FREQUENCY,
    hclk: DEFAULT_FREQUENCY,
};

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct Clocks {
    pub sysclk: Hertz,
    /// Clock of AHB
    pub hclk: Hertz,
}

#[inline]
pub fn clocks() -> &'static Clocks {
    unsafe { &CLOCKS }
}

pub(crate) fn init() {
    let flash = unsafe { &*pac::FLASH::PTR };
    let rcc = unsafe { &*pac::RCC::PTR };

    // SystemInit
    rcc.ctlr().modify(|_, w| w.hsion().set_bit());

    flash.actlr().modify(|_, w| w.latency().variant(0b10)); // 2 等待（24MHz<HCLK<=48MHz）

    // set hckl = sysclk = APB1
    rcc.cfgr0().modify(|_, w| w.hpre().variant(0));

    unsafe {
        CLOCKS = Clocks {
            sysclk: HSI_FREQUENCY,
            hclk: HSI_FREQUENCY,
        };
    }
}
