use crate::clock::{Clock, Hz};
use core::marker::PhantomData;
use samv71_pac::SUPC;

/// SlckSource is implemented by valid SLCK clock sources
pub trait SlckSource {}

/// SLCK RC oscillator
pub struct SlowRc {}

impl Clock for SlowRc {
    const FREQ: Hz = 32768;
}

impl SlckSource for SlowRc {}

/// SLCK internal 32.768 kHz oscillator
pub struct Xtal32Int {}

impl Clock for Xtal32Int {
    const FREQ: Hz = 32768;
}

impl SlckSource for Xtal32Int {}

/// SLCK external 32.768 kHz oscillator
pub struct Xtal32Ext {}

impl Clock for Xtal32Ext {
    const FREQ: Hz = 32768;
}

impl SlckSource for Xtal32Ext {}

/// The SAMV71 SLCK clock generator. If the XTAL source was selected by the program
/// it cannot be reset to using the RC source without power-cycling the MCU. This
/// is reflected in the typstate pattern of Slck.
pub struct Slck<S: Clock + SlckSource> {
    source: PhantomData<S>,
}

/// The SLCK generator does not modify the source clock frequency so we rexport it here.
impl<S: Clock + SlckSource> Clock for Slck<S> {
    const FREQ: Hz = S::FREQ;
}

impl Slck<SlowRc> {
    /// Returns the default SLCK. Only valid if called once.
    pub fn default() -> Self {
        Self {
            source: PhantomData,
        }
    }

    /// Private function to set SLCK source to XTAL.
    fn set_source_xtal32() {
        let supc = unsafe { &*SUPC::PTR };

        supc.cr().write(|w| {
            w.xtalsel().crystal_sel(); // Select xtal oscillator source
            w.key().passwd()
        });

        // Spin until osc selection is complete
        while supc.sr().read().oscsel().is_cryst() {}
    }

    /// Set SLCK to be driven by the internal 32.768 kHz crystal oscillator.
    /// Setting the SLCK source to XTAL requires powercycling.
    pub fn set_source_xtal32_internal(self) -> Slck<Xtal32Int> {
        Self::set_source_xtal32();

        Slck {
            source: PhantomData,
        }
    }

    /// Set SLCK to be driven by the external 32.768 kHz crystal oscillator.
    /// Setting the SLCK source to XTAL requires powercycling.
    pub fn set_source_xtal32_external(self) -> Slck<Xtal32Ext> {
        let supc = unsafe { &*SUPC::PTR };

        supc.mr().modify(|_, w| {
            w.oscbypass().bypass(); // Bypass internal xtal oscilator
            w.key().passwd()
        });

        Self::set_source_xtal32();

        Slck {
            source: PhantomData,
        }
    }
}
