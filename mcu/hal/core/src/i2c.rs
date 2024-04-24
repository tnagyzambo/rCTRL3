use crate::pio::{PeripheralA, PeripheralC, Pin, PA3, PA4, PB4, PB5, PD27, PD28};
use core::marker::PhantomData;
use samv71_pac::twihs0::RegisterBlock;
use samv71_pac::{Interrupt, NVIC, PMC, TWIHS0, TWIHS1, TWIHS2};

pub struct I2C<I: I2CId, M: I2CMode> {
    i2c: PhantomData<I>,
    mode: PhantomData<M>,
    _sda: I::SDA,
    _scl: I::SCL,
}

impl<I: I2CId> I2C<I, Host> {
    pub fn init(sda: I::SDA, scl: I::SCL) -> Self {
        // Enable clock to TWIHS peripheral
        Self::enable_peripheral();

        let i2c = unsafe { &*I::REG };

        // Hard-coded bus speed
        // TODO: Make variable
        i2c.cwgr().write(|w| {
            unsafe { w.ckdiv().bits(6) };
            unsafe { w.chdiv().bits(1) };
            unsafe { w.cldiv().bits(1) }
        });

        i2c.cr().write(|w| {
            w.svdis().set_bit(); // Disable client mode
            w.msen().set_bit() // Enable host mode
        });

        Self {
            i2c: PhantomData,
            mode: PhantomData,
            _sda: sda,
            _scl: scl,
        }
    }

    pub fn as_device<'a, D: I2CDevice<'a, I>>(&'a mut self, addr: u8) -> D {
        // Address is u7
        defmt::assert!(addr < 0xFF, "I2C DEVICE ADDR {:#x} OUT OF RANGE", addr);

        let i2c = unsafe { &*I::REG };
        i2c.mmr().write(|w| {
            unsafe { w.dadr().bits(addr) }; // Device I2C address
            w.iadrsz().bits(D::ADDR_MODE as u8) // Device internal addressing mode
        });

        D::new(self)
    }

    fn read(&self, reg: u8) -> u8 {
        let i2c = unsafe { &*I::REG };
        i2c.mmr().modify(|_, w| w.mread().set_bit()); // Select read operation
                                                      // TODO: Accomodate different iadr modes
        i2c.iadr().write(|w| unsafe { w.iadr().bits(reg as u32) }); // Set internal address
        i2c.cr().write(|w| w.start().set_bit()); // Start read operation
        while i2c.sr().read().rxrdy().bit_is_clear() {} // Spin until completion
        i2c.rhr().read().rxdata().bits() // Read RHR buffer
    }

    fn write(&self, reg: u8, byte: u8) {
        let i2c = unsafe { &*I::REG };
        i2c.mmr().modify(|_, w| w.mread().clear_bit()); // Select write operation
                                                        // TODO: Accomodate different iadr modes
        i2c.iadr().write(|w| unsafe { w.iadr().bits(reg as u32) }); // Set internal address
        i2c.thr().write(|w| unsafe { w.txdata().bits(byte) }); // Write data into Tx buffer
        i2c.cr().write(|w| w.stop().set_bit()); // Send STOP bit
        while i2c.sr().read().txrdy().bit_is_clear() {} // Spin while waiting for Tx
        while i2c.sr().read().txcomp().bit_is_clear() {} // Spin until completion
    }
}

impl<I: I2CId, M: I2CMode> I2C<I, M> {
    fn enable_peripheral() {
        // Enable clock to TWIHS peripheral
        let pmc = unsafe { &*PMC::PTR };
        if I::PER <= 31 {
            pmc.pcer0().write(|w| unsafe { w.bits(1 << I::PER) });
        } else {
            pmc.pcer1().write(|w| unsafe { w.bits(1 << (I::PER - 32)) });
        }
    }

    pub fn enable_interrupts(&self) {
        unsafe { NVIC::unmask(I::INT) }; // Enable twihs interrupts in the NVIC
        NVIC::unpend(I::INT); // Unpend twihs interrupts

        let i2c = unsafe { &*I::REG };
        let _ = i2c.sr().read().bits(); // Clear interrupt status register
    }

    // TODO: Make this better
    pub fn interrupts(&self) -> u32 {
        let i2c = unsafe { &*I::REG };
        i2c.sr().read().bits()
    }
}

//
//
//

pub trait I2CMode {}

pub struct Host {}

impl I2CMode for Host {}

pub struct Client {}

impl I2CMode for Client {}

//
//
//

pub trait I2CDevice<'a, I: I2CId> {
    const ADDR_MODE: I2CInternalAddrMode;
    fn new(i2c: &'a I2C<I, Host>) -> Self;
}

pub enum I2CInternalAddrMode {
    ThreeByte = 3,
    TwoByte = 2,
    OneByte = 1,
    None = 0,
}

//
//
//

pub struct LSM9DS1AccelerometerConfig {}

pub struct LSM9DS1Accelerometer<'a, I: I2CId> {
    i2c: &'a I2C<I, Host>,
}

impl<'a, I: I2CId> I2CDevice<'a, I> for LSM9DS1Accelerometer<'a, I> {
    const ADDR_MODE: I2CInternalAddrMode = I2CInternalAddrMode::OneByte;
    fn new(i2c: &'a I2C<I, Host>) -> Self {
        Self { i2c }
    }
}

impl<'a, I: I2CId> LSM9DS1Accelerometer<'a, I> {
    // 0x00 reserved
    // 0x01 reserved
    // 0x02 reserved
    // 0x03 reserved
    const ACT_THS: u8 = 0x04;
    const ACT_DUR: u8 = 0x05;
    const INT_GEN_CFG_XL: u8 = 0x06;
    const INT_GEN_THS_X_XL: u8 = 0x07;
    const INT_GEN_THS_Y_XL: u8 = 0x08;
    const INT_GEN_THS_Z_XL: u8 = 0x09;
    const INT_GEN_DUR_XL: u8 = 0x0A;
    const REFERENCE_G: u8 = 0x0B;
    const INT1_CTRL: u8 = 0xC;
    const INT2_CTRL: u8 = 0xD;
    // 0x0E reserved
    const WHO_AM_I: u8 = 0x0F;
    const CTRL_REG1_G: u8 = 0x10;
    const CTRL_REG2_G: u8 = 0x11;
    const CTRL_REG3_G: u8 = 0x12;
    const ORIENT_CFG_G: u8 = 0x13;
    const INT_GEN_SRC_G: u8 = 0x14;
    const OUT_TEMP_L: u8 = 0x15;
    const OUT_TEMP_H: u8 = 0x16;
    const STATUS_REG1: u8 = 0x17;
    const OUT_X_L_G: u8 = 0x18;
    const OUT_X_H_G: u8 = 0x19;
    const OUT_Y_L_G: u8 = 0x1A;
    const OUT_Y_H_G: u8 = 0x1B;
    const OUT_Z_L_G: u8 = 0x1C;
    const OUT_Z_H_G: u8 = 0x1D;
    const CTRL_REG4: u8 = 0x1E;
    const CTRL_REG5_XL: u8 = 0x1F;
    const CTRL_REG6_XL: u8 = 0x20;
    const CTRL_REG7_XL: u8 = 0x21;
    const CTRL_REG8: u8 = 0x22;
    const CTRL_REG9: u8 = 0x23;
    const CTRL_REG10: u8 = 0x24;
    // 0x25 reserved
    const INT_GEN_SRC_XL: u8 = 0x26;
    const STATUS_REG2: u8 = 0x27;
    const OUT_X_L_XL: u8 = 0x28;
    const OUT_X_H_XL: u8 = 0x29;
    const OUT_Y_L_XL: u8 = 0x2A;
    const OUT_Y_H_XL: u8 = 0x2B;
    const OUT_Z_L_XL: u8 = 0x2C;
    const OUT_Z_H_XL: u8 = 0x2D;
    const FIFO_CTRL: u8 = 0x2E;
    const FIFO_SRC: u8 = 0x2F;
    const INT_GEN_CFG_G: u8 = 0x30;
    const INT_GEN_THS_XH_G: u8 = 0x31;
    const INT_GEN_THS_XL_G: u8 = 0x32;
    const INT_GEN_THS_YH_G: u8 = 0x33;
    const INT_GEN_THS_YL_G: u8 = 0x34;
    const INT_GEN_THS_ZH_G: u8 = 0x35;
    const INT_GEN_THS_ZL_G: u8 = 0x36;
    const INT_GEN_DUR_G: u8 = 0x37;

    pub fn gyro_enable(&self) {
        self.i2c.write(Self::CTRL_REG1_G, 0b01000000);
    }

    pub fn gyro_x(&self) -> f32 {
        let h = self.i2c.read(Self::OUT_X_H_G);
        let l = self.i2c.read(Self::OUT_X_L_G);

        // Default 245 dps
        let gyro = Self::twos_compliment::<16>(h, l);
        gyro as f32 * 0.00875
    }

    pub fn gyro_y(&self) -> f32 {
        let h = self.i2c.read(Self::OUT_Y_H_G);
        let l = self.i2c.read(Self::OUT_Y_L_G);

        // Default 245 dps
        let gyro = Self::twos_compliment::<16>(h, l);
        gyro as f32 * 0.00875
    }

    pub fn gyro_z(&self) -> f32 {
        let h = self.i2c.read(Self::OUT_Z_H_G);
        let l = self.i2c.read(Self::OUT_Z_L_G);

        // Default 245 dps
        let gyro = Self::twos_compliment::<16>(h, l);
        gyro as f32 * 0.00875
    }

    fn twos_compliment<const L: usize>(h: u8, l: u8) -> i16 {
        //let value = ((h as u16) << 8) | l as u16;
        //if value & (1 << (L - 1)) != 0 {
        //    // Handle negative numbers
        //    (value | (0x10000 >> (L - 16)) as u16) as i16
        //} else {
        //    value as i16
        //}
        let value = ((h as u16) << 8) | l as u16;
        if value & 0x8000 != 0 {
            // Handle negative numbers
            (value | 0x0000) as i16
        } else {
            value as i16
        }
    }
}

//
//
//

pub trait I2CId {
    const REG: *const RegisterBlock;
    const INT: Interrupt;
    const PER: u8;
    type SDA;
    type SCL;
}

pub struct I2C0 {}

impl I2CId for I2C0 {
    const REG: *const RegisterBlock = TWIHS0::PTR;
    const INT: Interrupt = Interrupt::TWIHS0;
    const PER: u8 = 19;
    type SDA = Pin<PA3, PeripheralA>;
    type SCL = Pin<PA4, PeripheralA>;
}

pub struct I2C1 {}

impl I2CId for I2C1 {
    const REG: *const RegisterBlock = TWIHS1::PTR;
    const INT: Interrupt = Interrupt::TWIHS1;
    const PER: u8 = 19;
    type SDA = Pin<PB4, PeripheralA>;
    type SCL = Pin<PB5, PeripheralA>;
}

pub struct I2C2 {}

impl I2CId for I2C2 {
    const REG: *const RegisterBlock = TWIHS2::PTR;
    const INT: Interrupt = Interrupt::TWIHS2;
    const PER: u8 = 41;
    type SDA = Pin<PD27, PeripheralC>;
    type SCL = Pin<PD28, PeripheralC>;
}
