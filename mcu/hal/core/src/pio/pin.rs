use core::marker::PhantomData;
use samv71_pac::pioa::RegisterBlock; // All PIO banks are type aliases of PIOA
use samv71_pac::{PIOA, PIOB, PIOC, PIOD, PIOE};

/// Pin is generic on all PinId's and use typestate pattern on PinMode to expose
/// certain functions.
pub struct Pin<P: PinId, M: PinMode> {
    pin: PhantomData<P>,
    mode: PhantomData<M>,
}

// Actions available in all configurations
impl<P: PinId, M: PinMode> Pin<P, M> {
    pub fn enable_pullup(&self) {
        let reg = unsafe { &*P::REG };
        reg.puer().write(|w| unsafe { w.bits(1 << P::ID) })
    }

    pub fn into_input(&mut self) -> Pin<P, Input> {
        Pin {
            pin: self.pin,
            mode: PhantomData,
        }
    }

    pub fn into_output(&mut self) -> Pin<P, Output> {
        Pin {
            pin: self.pin,
            mode: PhantomData,
        }
    }

    pub fn into_peripheral<PER: PeripheralId + PinMode>(&mut self) -> Pin<P, PER> {
        Pin {
            pin: self.pin,
            mode: PhantomData,
        }
    }
}

// Actions specific to Pins configured as inputs
impl<P: PinId> Pin<P, Input> {
    pub fn init() -> Self {
        let reg = unsafe { &*P::REG };
        reg.per().write(|w| unsafe { w.bits(1 << P::ID) }); // Disable peripheral mode
        reg.odr().write(|w| unsafe { w.bits(1 << P::ID) }); // Disable output

        Self {
            pin: PhantomData,
            mode: PhantomData,
        }
    }

    /// Disable input filtering.
    pub fn disable_input_filter(&self) {
        let reg = unsafe { &*P::REG };
        reg.ifdr().write(|w| unsafe { w.bits(1 << P::ID) }); // Disable input filter
    }

    /// Filters pulses with a duration of less than 1/2 of a period programmed into the PIO bank.
    /// This is mutually exlusive with enable_glitch_filter().
    pub fn enable_debounce_filter(&self) {
        let reg = unsafe { &*P::REG };
        reg.ifscer().write(|w| unsafe { w.bits(1 << P::ID) }); // Select debounce filter
        reg.ifer().write(|w| unsafe { w.bits(1 << P::ID) }); // Enable input filter
    }

    /// Filters glitches with a duration of less than 1/2 Host clock period.
    /// This is mutually exlusive with enable_debounce_filter().
    pub fn enable_glitch_filter(&self) {
        let reg = unsafe { &*P::REG };
        reg.ifscdr().write(|w| unsafe { w.bits(1 << P::ID) }); // Select glitch filter
        reg.ifer().write(|w| unsafe { w.bits(1 << P::ID) }); // Enable input filter
    }

    pub fn enable_rising_edge_interrupt(&self) {
        let reg = unsafe { &*P::REG };
        reg.aimer().write(|w| unsafe { w.bits(1 << P::ID) }); // Enable additional interrupt modes
        reg.esr().write(|w| unsafe { w.bits(1 << P::ID) }); // Enable edgedetection
        reg.rehlsr().write(|w| unsafe { w.bits(1 << P::ID) }); // Enable rising edge detection
        reg.ier().write(|w| unsafe { w.bits(1 << P::ID) }); // Enable input change interrupt
    }
}

impl<P: PinId> Pin<P, Output> {
    pub fn init() -> Self {
        let reg = unsafe { &*P::REG };
        reg.per().write(|w| unsafe { w.bits(1 << P::ID) }); // Disable peripheral mode
        reg.oer().write(|w| unsafe { w.bits(1 << P::ID) }); // Enable output

        Self {
            pin: PhantomData,
            mode: PhantomData,
        }
    }

    pub fn set_high(&self) {
        let reg = unsafe { &*P::REG };
        reg.sodr().write(|w| unsafe { w.bits(1 << P::ID) }); // Write ouput high
    }

    pub fn set_low(&self) {
        let reg = unsafe { &*P::REG };
        reg.codr().write(|w| unsafe { w.bits(1 << P::ID) }); // Write ouput low
    }

    pub fn toggle(&self) {
        let reg = unsafe { &*P::REG };
        if (reg.odsr().read().bits() & (1 << P::ID) as u32) != 0 {
            self.set_low();
        } else {
            self.set_high();
        }
    }
}

impl<P: PinId, M: PinMode + PeripheralId> Pin<P, M> {
    pub fn init() -> Self {
        let reg = unsafe { &*P::REG };

        // Select peripheral mode
        reg.abcdsr(0)
            .modify(|_, w| unsafe { w.bits((M::ABCDSR0 << P::ID) as u32) });
        reg.abcdsr(1)
            .modify(|_, w| unsafe { w.bits((M::ABCDSR1 << P::ID) as u32) });
        reg.pdr().write(|w| unsafe { w.bits(1 << P::ID) }); // Enable peripheral mode

        Self {
            pin: PhantomData,
            mode: PhantomData,
        }
    }
}

//
//
//

/// PinMode trait allows Pin to implement a typestate pattern on
/// a handful of primary configurations such as Input, Output and Peripheral modes.
pub trait PinMode {}

/// Pin input configuration
pub struct Input {}

impl PinMode for Input {}

/// Pin output configuration
pub struct Output {}

impl PinMode for Output {}

//
//
//

/// PeripheralId trait allows Peripheral to retrieve the correct values to set
/// PIO registers ABCDSR0 and ABCDSR1 to select the desired peripheral function.
pub trait PeripheralId {
    const ABCDSR0: u8;
    const ABCDSR1: u8;
}

/// Pin peripheral mode A configuration.
pub struct PeripheralA {}

impl PeripheralId for PeripheralA {
    const ABCDSR0: u8 = 0;
    const ABCDSR1: u8 = 0;
}

impl PinMode for PeripheralA {}

/// Pin peripheral mode B configuration.
pub struct PeripheralB {}

impl PeripheralId for PeripheralB {
    const ABCDSR0: u8 = 1;
    const ABCDSR1: u8 = 0;
}

impl PinMode for PeripheralB {}

/// Pin peripheral mode C configuration.
pub struct PeripheralC {}

impl PeripheralId for PeripheralC {
    const ABCDSR0: u8 = 0;
    const ABCDSR1: u8 = 1;
}

impl PinMode for PeripheralC {}

/// Pin peripheral mode D configuration.
pub struct PeripheralD {}

impl PeripheralId for PeripheralD {
    const ABCDSR0: u8 = 1;
    const ABCDSR1: u8 = 1;
}

impl PinMode for PeripheralD {}

//
//
//

/// PinId trait allows Pin to retrieve the register and Id mapped to
/// a particular pin.
pub trait PinId {
    const REG: *const RegisterBlock;
    const ID: u8;
}

// TODO: Make bellow section into a macro
pub struct PA3 {}

impl PinId for PA3 {
    const REG: *const RegisterBlock = PIOA::PTR;
    const ID: u8 = 3;
}

pub struct PA4 {}

impl PinId for PA4 {
    const REG: *const RegisterBlock = PIOA::PTR;
    const ID: u8 = 4;
}

pub struct PA9 {}

impl PinId for PA9 {
    const REG: *const RegisterBlock = PIOA::PTR;
    const ID: u8 = 9;
}

pub struct PA23 {}

impl PinId for PA23 {
    const REG: *const RegisterBlock = PIOA::PTR;
    const ID: u8 = 23;
}

pub struct PB1 {}

impl PinId for PB1 {
    const REG: *const RegisterBlock = PIOB::PTR;
    const ID: u8 = 1;
}
