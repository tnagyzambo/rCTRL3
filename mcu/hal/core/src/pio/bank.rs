use core::marker::PhantomData;
use hal_macro::{BitFieldIterator, TryFromU8};
use samv71_pac::pioa::RegisterBlock;
use samv71_pac::{Interrupt, NVIC, PIOA, PIOB, PIOC, PIOD, PIOE, PMC}; // All PIO banks are type aliases of PIOA

#[repr(u8)]
#[derive(TryFromU8, BitFieldIterator)]
pub enum PioInterrupt {
    P31 = 31,
    P30 = 30,
    P29 = 29,
    P28 = 28,
    P27 = 27,
    P26 = 26,
    P25 = 25,
    P24 = 24,
    P23 = 23,
    P22 = 22,
    P21 = 21,
    P20 = 20,
    P19 = 19,
    P18 = 18,
    P17 = 17,
    P16 = 16,
    P15 = 15,
    P14 = 14,
    P13 = 13,
    P12 = 12,
    P11 = 11,
    P10 = 10,
    P9 = 9,
    P8 = 8,
    P7 = 7,
    P6 = 6,
    P5 = 5,
    P4 = 4,
    P3 = 3,
    P2 = 2,
    P1 = 1,
    P0 = 0,
}

pub struct Bank<B: BankId> {
    bank: PhantomData<B>,
}

impl<B: BankId> Bank<B> {
    pub fn init() -> Self {
        // Enable clock to PIO bank
        // All PIO banks are controlled by the PCER0 register
        // so we can do this
        let pmc = unsafe { &*PMC::PTR };
        pmc.pcer0().write(|w| unsafe { w.bits(1 << B::PER) });

        Self { bank: PhantomData }
    }

    pub fn enable_interrupts(&self) {
        unsafe { NVIC::unmask(B::INT) }; // Enable bank interrupts in the NVIC
        NVIC::unpend(B::INT); // Unpend bank interrupts

        let reg = unsafe { &*B::REG };
        let _ = reg.isr().read().bits(); // Clear interrupt status register
    }

    pub fn interrupts(&self) -> PioInterruptIterator {
        let reg = unsafe { &*B::REG };
        PioInterruptIterator::new(reg.isr().read().bits())
    }

    /// Set the division of the bank's debounce filter.
    /// The period is calculated by:
    ///
    /// t_debounce = ((div + 1) * 2) * t_slck
    pub fn set_debounce_period(&self, div: u32) {
        let reg = unsafe { &*B::REG };
        reg.scdr().write(|w| unsafe { w.bits(div) }); // Set slow clock divider
    }
}

/// BankId allows Bank to retrieve the register associated with a
/// particular bank.
pub trait BankId {
    const REG: *const RegisterBlock;
    const INT: Interrupt;
    const PER: u8;
}

pub struct BankA {}

impl BankId for BankA {
    const REG: *const RegisterBlock = PIOA::PTR;
    const INT: Interrupt = Interrupt::PIOA;
    const PER: u8 = 10;
}

pub struct BankB {}

impl BankId for BankB {
    const REG: *const RegisterBlock = PIOB::PTR;
    const INT: Interrupt = Interrupt::PIOB;
    const PER: u8 = 11;
}

pub struct BankC {}

impl BankId for BankC {
    const REG: *const RegisterBlock = PIOC::PTR;
    const INT: Interrupt = Interrupt::PIOC;
    const PER: u8 = 12;
}

pub struct BankD {}

impl BankId for BankD {
    const REG: *const RegisterBlock = PIOD::PTR;
    const INT: Interrupt = Interrupt::PIOD;
    const PER: u8 = 16;
}

pub struct BankE {}

impl BankId for BankE {
    const REG: *const RegisterBlock = PIOE::PTR;
    const INT: Interrupt = Interrupt::PIOE;
    const PER: u8 = 17;
}
