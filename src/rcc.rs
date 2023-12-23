use fugit::HertzU32 as Hertz;

const HSI_FREQUENCY: Hertz = Hertz::from_raw(48_000_000);

static mut CLOCKS: Clocks = Clocks {
    // Power on default
    sysclk: HSI_FREQUENCY,
    hclk: HSI_FREQUENCY,
};

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct Clocks {
    pub sysclk: Hertz,
    pub hclk: Hertz,
}

#[inline]
pub fn clocks() -> &'static Clocks {
    unsafe { &CLOCKS }
}
