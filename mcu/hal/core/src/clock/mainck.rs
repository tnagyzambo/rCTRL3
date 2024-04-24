use crate::clock::{Clock, Hz};
use core::marker::PhantomData;
use samv71_pac::PMC;

/// MainckSource is implemented by valid MAINCK clock sources.
pub trait MainckSource {}

/// MAINCK RC oscillator
pub struct MainRc<const F: usize> {}

impl<const F: usize> Clock for MainRc<F> {
    const FREQ: Hz = F;
}

impl<const F: usize> MainckSource for MainRc<F> {}

/// MAINCK internal 12 MHz oscillator
pub struct XtalInt {}

impl Clock for XtalInt {
    const FREQ: Hz = 12000000;
}

impl MainckSource for XtalInt {}

/// MAINCK external oscillator.
/// Valid frequencies are between 3 MHz and 20 MHz.
pub struct XtalExt<const F: usize> {}

impl<const F: usize> Clock for XtalExt<F> {
    const FREQ: Hz = F;
}

impl<const F: usize> MainckSource for XtalExt<F> {}

/// The SAMV71 MAINCK clock generator. This is freely configurable.
pub struct Mainck<S: MainckSource> {
    source: PhantomData<S>,
}

/// The MAICK generator does not modify the source clock frequency so we rexport it here.
impl<S: Clock + MainckSource> Clock for Mainck<S> {
    const FREQ: Hz = S::FREQ;
}

impl Mainck<MainRc<12000000>> {
    /// Returns the default MAINCK. Only valid once.
    pub fn default() -> Self {
        Self {
            source: PhantomData,
        }
    }
}

impl<const F: usize> Mainck<MainRc<F>> {
    /// Set RC frequency to 4 MHz.
    pub fn set_freq_4mhz() -> Mainck<MainRc<4000000>> {
        let pmc = unsafe { &*PMC::PTR };

        // Set main RC frequency to 4 MHz
        pmc.ckgr_mor().modify(|_, w| w.moscrcf()._4_mhz());

        // Spin until RC frequency has stabilized
        while pmc.sr().read().moscrcs().bit_is_clear() {}

        Mainck {
            source: PhantomData,
        }
    }

    /// Set RC frequency to 8 MHz.
    pub fn set_freq_8mhz() -> Mainck<MainRc<8000000>> {
        let pmc = unsafe { &*PMC::PTR };

        // Set main RC frequency to 8 MHz
        pmc.ckgr_mor().modify(|_, w| w.moscrcf()._8_mhz());

        // Spin until RC frequency has stabilized
        while pmc.sr().read().moscrcs().bit_is_clear() {}

        Mainck {
            source: PhantomData,
        }
    }

    /// Set RC frequency to 12 MHz.
    pub fn set_freq_12mhz() -> Mainck<MainRc<12000000>> {
        let pmc = unsafe { &*PMC::PTR };

        // Set main RC frequency to 12 MHz
        pmc.ckgr_mor().modify(|_, w| w.moscrcf()._12_mhz());

        // Spin until RC frequency has stabilized
        while pmc.sr().read().moscrcs().bit_is_clear() {}

        Mainck {
            source: PhantomData,
        }
    }

    /// Private function to set MAINCK source to XTAL.
    fn set_source_xtal(wakeup: u8) {
        let pmc = unsafe { &*PMC::PTR };

        pmc.ckgr_mor().write(|w| {
            w.moscxten().set_bit(); // Enable main xtal oscilator
            unsafe { w.moscxtst().bits(wakeup) } // Set main crystal osc wake up time
        });

        // Spin until xtal frequency has stabilized
        while pmc.sr().read().moscxts().bit_is_clear() {}

        // Switch MAINCK to main crystal osc
        pmc.ckgr_mor().write(|w| w.moscsel().set_bit());

        // Spin until MAICK selection is complete
        while pmc.sr().read().moscsels().bit_is_clear() {}
    }

    /// Set MAINCK source to the internal 12 MHz crystal oscillator.
    pub fn set_source_xtal_internal(self) -> Mainck<XtalInt> {
        Self::set_source_xtal(0xFF); // Set xtal source with 62ms wakeup

        Mainck {
            source: PhantomData,
        }
    }

    /// Set MAINCK source to the external crystal ocillator (if present).
    /// The external crystal can have a freq between 3 MHz and 20 MHz.
    pub fn set_source_xtal_external<const F_XTAL: usize>(self) -> Mainck<XtalExt<F_XTAL>> {
        let pmc = unsafe { &*PMC::PTR };

        // Bypass internal xtal oscilator
        pmc.ckgr_mor().modify(|_, w| w.moscxtby().set_bit());

        Self::set_source_xtal(0); // Must use 0ms wake up in bypass mode

        Mainck {
            source: PhantomData,
        }
    }
}
