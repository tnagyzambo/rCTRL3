use crate::clock::{Clock, Hz, Mainck, MainckSource, Slck, SlckSource};
use core::marker::PhantomData;
use samv71_pac::PMC;

/// MckSource is implemented by valid MCK clock sources.
pub trait MckSource {
    const CSS: u8;
}

/// SLCK
impl<S: Clock + SlckSource> MckSource for Slck<S> {
    const CSS: u8 = 0;
}

/// MAINCK
impl<S: Clock + MainckSource> MckSource for Mainck<S> {
    const CSS: u8 = 1;
}

/// MCK clock. This is free configurable.
pub struct Mck<S: Clock + MckSource, P: MckPres, D: MckDiv> {
    source: PhantomData<S>,
    pres: PhantomData<P>,
    div: PhantomData<D>,
}

impl<S: Clock + MckSource, P: MckPres, D: MckDiv> From<S> for Mck<S, P, D> {
    fn from(_source: S) -> Self {
        let pmc = unsafe { &*PMC::PTR };

        // Select MCK source
        pmc.mckr().modify(|_, w| w.css().bits(S::CSS));

        // Spin until MCK is ready
        while pmc.sr().read().mckrdy().bit_is_clear() {}

        pmc.mckr().modify(|_, w| {
            w.pres().bits(P::PRES); // Set clock source prescaler division
            w.mdiv().bits(D::MDIV) // Set output host clock division
        });

        Self {
            source: PhantomData,
            pres: PhantomData,
            div: PhantomData,
        }
    }
}

impl<S: Clock + MckSource, P: MckPres, D: MckDiv> Clock for Mck<S, P, D> {
    const FREQ: Hz = S::FREQ / P::VAL / D::VAL;
}

/// New type for a division with a const value
pub struct Div<const V: usize> {}

/// Val trait to describe acess to the const value
pub trait Val {
    const VAL: usize;
}

/// Generic trait implementation to access Div<V>'s const value
impl<const V: usize> Val for Div<V> {
    const VAL: usize = V;
}

/// Super trait Val to describe valid Div values
pub trait MckPres: Val {
    const PRES: u8;
}
pub trait MckDiv: Val {
    const MDIV: u8;
}

/// Implementations of valid Div values for MCK PRES and MDIV registers
impl MckPres for Div<1> {
    const PRES: u8 = 0;
}
impl MckDiv for Div<1> {
    const MDIV: u8 = 0;
}

//
impl MckPres for Div<2> {
    const PRES: u8 = 1;
}
impl MckDiv for Div<2> {
    const MDIV: u8 = 1;
}

//
impl MckPres for Div<3> {
    const PRES: u8 = 7;
}
impl MckDiv for Div<3> {
    const MDIV: u8 = 3;
}

//
impl MckPres for Div<4> {
    const PRES: u8 = 2;
}
impl MckDiv for Div<4> {
    const MDIV: u8 = 2;
}

//
impl MckPres for Div<8> {
    const PRES: u8 = 3;
}

//
impl MckPres for Div<16> {
    const PRES: u8 = 4;
}

//
impl MckPres for Div<32> {
    const PRES: u8 = 5;
}

//
impl MckPres for Div<64> {
    const PRES: u8 = 6;
}
