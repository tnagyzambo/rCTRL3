use embedded_time::{
    clock::{Clock, Error},
    fraction::Fraction,
    Instant,
};
use hal_macro::{BitFieldIterator, TryFromU8};
use samv71_pac::{Interrupt, NVIC, RTT};

#[repr(u8)]
#[derive(TryFromU8, BitFieldIterator)]
pub enum RttInterrupt {
    RTTINC = 1,
    ALMS = 0,
}

pub struct Rtt {
    pub rtt: RTT,
    f_slck: u32,
    res: u32,
}

impl Rtt {
    pub fn new(rtt: RTT, f_slck: u32, res: u32) -> Self {
        unsafe {
            NVIC::unmask(Interrupt::RTT); // Enable RTT interrupt in the NVIC
            NVIC::unpend(Interrupt::RTT); // Unpend RTT interrupts
        }

        // Setup RTT
        rtt.mr().modify(
            |_, w| w.rttincien().clear_bit(), // RTTINCIEN must be cleared prior to setting RTPRES
        );

        rtt.mr().modify(|_, w| {
            w.rttrst().set_bit(); // Reset RTT
            w.almien().set_bit();
            unsafe { w.rtpres().bits(res as u16) } // Set RTT period to 91.55us (RTPRES * SLCK)
        });

        Self { rtt, f_slck, res }
    }

    pub fn millis(&self) -> u32 {
        // The value register must be read twice at the same value to be a valid read
        let mut prev_read = self.rtt.vr().read().bits();

        // TODO: Add timeout
        loop {
            let cur_read = self.rtt.vr().read().bits();

            if cur_read == prev_read {
                return (cur_read as f64 * 0.09155) as u32;
            }

            prev_read = cur_read;
        }
    }

    pub fn interrupts(&self) -> RttInterruptIterator {
        RttInterruptIterator::new(self.rtt.sr().read().bits())
    }
}

impl Clock for Rtt {
    type T = u32;

    const SCALING_FACTOR: Fraction = Fraction::new(0, 0);

    fn try_now(&self) -> Result<Instant<Self>, Error> {
        Err(Error::Unspecified)
    }
}
