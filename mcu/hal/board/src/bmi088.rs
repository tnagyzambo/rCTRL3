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
        self.write(0x8000 | (0x00 << 8));
        self.read() as u8
    }

    fn set_range(&self) {
        // Should set to +/-250 deg/s
        self.write((0x0F << 8) | 0x03);
        self.read();
    }

    fn set_bw(&self) {
        // Should set to 32 Hz
        self.write((0x10 << 8) | 0x07);
        self.read();
    }

    fn gyro_x(&self) -> f32 {
        self.write(0x8000 | (0x02 << 8));
        let lsb = self.read() as u8;
        self.write(0x8000 | (0x03 << 8));
        let msb = self.read() as u8;
        let b = (msb as u16 * 256 + lsb as u16) as i16;
        (b as f32 / 32767.0) * 250.0
    }

    fn gyro_y(&self) -> f32 {
        self.write(0x8000 | (0x04 << 8));
        let lsb = self.read() as u8;
        self.write(0x8000 | (0x05 << 8));
        let msb = self.read() as u8;
        let b = (msb as u16 * 256 + lsb as u16) as i16;
        (b as f32 / 32767.0) * 250.0
    }

    fn gyro_z(&self) -> f32 {
        self.write(0x8000 | (0x06 << 8));
        let lsb = self.read() as u8;
        self.write(0x8000 | (0x07 << 8));
        let msb = self.read() as u8;
        let b = (msb as u16 * 256 + lsb as u16) as i16;
        (b as f32 / 32767.0) * 250.0
    }
}

impl<D, C: ChipSelect> BMI088<C> for D where D: SpiDevice<C> {}
