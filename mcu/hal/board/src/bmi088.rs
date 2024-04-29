use core::marker::PhantomData;
use hal_core::spi::*;

pub struct BMI088Config {}

impl BMI088Config {
    pub fn default() -> Self {
        Self {}
    }
}

pub trait BMI088<C: ChipSelect>: SpiDevice<C> {
    fn gyro_id(&self) -> u8 {
        self.write(0x8000);
        (self.read() >> 1) as u8
    }
}

impl<D, C: ChipSelect> BMI088<C> for D where D: SpiDevice<C> {}
