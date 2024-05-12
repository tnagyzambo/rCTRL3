pub use accl::Accelerometer;
pub use gyro::Gyro;

pub mod accl {
    use hal_core::spi::*;

    const G: f32 = 9.807;

    pub struct Accelerometer<'a> {
        ctx: &'a mut Context,
        read: fn() -> u16,
        write: fn(u16, bool),
    }

    impl<'a> Accelerometer<'a> {
        const CHIP_ID: u8 = 0x1E;

        pub fn from<H: SpiDevice<C>, C: ChipSelect>(ctx: &'a mut Context, _spi: &H) -> Self {
            Self {
                ctx,
                read: H::read,
                write: H::write,
            }
        }

        fn read(&self, reg: Register) -> u8 {
            (self.write)(0x8000 | ((reg as u16) << 8), false);
            (self.read)();
            (self.write)(0x00, true);
            ((self.read)() >> 8) as u8
        }

        fn write(&self, reg: Register, data: u8) {
            (self.write)(((reg as u16) << 8) | data as u16, false);
            (self.read)();
            (self.write)(0x00, true);
            (self.read)();
        }

        /// Initialize the BMI088 accelerometer for SPI communication.
        ///
        /// Perform a dummy read of the ACC_CHIP_ID register to generate a rising edge
        /// on the chip select line of the accelerometer. This switches the BMI088 to SPI
        /// communication until the power cycle.
        pub fn init_spi(&self) {
            self.read(Register::ChipId);
        }

        pub fn enable(&mut self) {
            self.write(Register::PwrCtrl, PwrCtrl::Enable as u8);
            self.ctx.pwr_ctrl = PwrCtrl::Enable;
        }

        pub fn disable(&mut self) {
            self.write(Register::PwrCtrl, PwrCtrl::Disable as u8);
            self.ctx.pwr_ctrl = PwrCtrl::Disable;
        }

        pub fn id_ok(&self) -> bool {
            Self::CHIP_ID == self.read(Register::ChipId)
        }

        pub fn set_range(&mut self, range: Range) {
            self.write(Register::Range, range as u8);
            self.ctx.range = range;
        }

        pub fn set_bwp(&mut self, bwp: Bwp) {
            self.write(Register::Conf, bwp as u8 | self.ctx.odr as u8);
            self.ctx.bwp = bwp;
        }

        pub fn set_odr(&mut self, odr: Odr) {
            self.write(Register::Conf, self.ctx.bwp as u8 | odr as u8);
            self.ctx.odr = odr;
        }

        pub fn x(&self) -> f32 {
            let lsb = self.read(Register::XLsb) as u16;
            let msb = self.read(Register::XMsb) as u16;
            let range: f32 = self.ctx.range.into();

            let b = (msb * 256 + lsb) as i16;
            (b as f32 / 32767.0) * range
        }

        pub fn y(&self) -> f32 {
            let lsb = self.read(Register::YLsb) as u16;
            let msb = self.read(Register::YMsb) as u16;
            let range: f32 = self.ctx.range.into();

            let b = (msb * 256 + lsb) as i16;
            (b as f32 / 32767.0) * range
        }

        pub fn z(&self) -> f32 {
            let lsb = self.read(Register::ZLsb) as u16;
            let msb = self.read(Register::ZMsb) as u16;
            let range: f32 = self.ctx.range.into();

            let b = (msb * 256 + lsb) as i16;
            (b as f32 / 32767.0) * range
        }
    }

    pub struct Context {
        pwr_ctrl: PwrCtrl,
        bwp: Bwp,
        odr: Odr,
        range: Range,
    }

    impl Context {
        pub fn default() -> Self {
            Self {
                pwr_ctrl: PwrCtrl::Disable,
                bwp: Bwp::Normal,
                odr: Odr::_100,
                range: Range::_6g,
            }
        }
    }

    #[allow(dead_code)]
    enum Register {
        SoftReset = 0x7E,
        PwrCtrl = 0x7D,
        PwrConf = 0x7C,
        SelfTest = 0x6D,
        IntMapData = 0x58,
        Int2IoCtrl = 0x54,
        Int1IoCtrl = 0x53,
        FifoConfig1 = 0x49,
        FifoConfig0 = 0x48,
        FifoWtm1 = 0x47,
        FifoWtm0 = 0x46,
        FifoDowns = 0x45,
        Range = 0x41,
        Conf = 0x40,
        FifoData = 0x26,
        FifoLength1 = 0x25,
        FifoLength0 = 0x24,
        TempLsb = 0x23,
        TempMsb = 0x22,
        IntStat = 0x1D,
        SensorTime2 = 0x1A,
        SensorTime1 = 0x19,
        SensorTime0 = 0x18,
        ZMsb = 0x17,
        ZLsb = 0x16,
        YMsb = 0x15,
        YLsb = 0x14,
        XMsb = 0x13,
        XLsb = 0x12,
        Status = 0x03,
        ErrReg = 0x02,
        ChipId = 0x00,
    }

    enum PwrCtrl {
        Enable = 0x04,
        Disable = 0x00,
    }

    #[derive(Clone, Copy)]
    pub enum Bwp {
        Normal = 0x0A << 4,
        OSR2 = 0x09 << 4,
        OSR4 = 0x08 << 4,
    }

    #[derive(Clone, Copy)]
    pub enum Odr {
        _1600 = 0x0C,
        _800 = 0x0B,
        _400 = 0x0A,
        _200 = 0x09,
        _100 = 0x08,
        _50 = 0x07,
        _25 = 0x06,
        _12_5 = 0x05,
    }

    #[derive(Clone, Copy)]
    pub enum Range {
        _24g = 0x03,
        _12g = 0x02,
        _6g = 0x01,
        _3g = 0x00,
    }

    impl Into<f32> for Range {
        fn into(self) -> f32 {
            match self {
                Self::_24g => 24.0 * G,
                Self::_12g => 12.0 * G,
                Self::_6g => 6.0 * G,
                Self::_3g => 3.0 * G,
            }
        }
    }
}

pub mod gyro {
    use hal_core::spi::*;

    pub struct Gyro<'a> {
        ctx: &'a mut Context,
        read: fn() -> u16,
        write: fn(u16, bool),
    }

    impl<'a> Gyro<'a> {
        const CHIP_ID: u8 = 0x0F;

        pub fn from<H: SpiDevice<C>, C: ChipSelect>(ctx: &'a mut Context, _spi: &H) -> Self {
            Self {
                ctx,
                read: H::read,
                write: H::write,
            }
        }

        fn read(&self, reg: Register) -> u8 {
            (self.write)(0x8000 | ((reg as u16) << 8), true);
            (self.read)() as u8
        }

        fn write(&self, reg: Register, data: u8) {
            (self.write)(((reg as u16) << 8) | data as u16, true);
            (self.read)();
        }

        pub fn id_ok(&self) -> bool {
            Self::CHIP_ID == self.read(Register::ChipId)
        }

        pub fn reset(&self) {
            self.write(Register::SoftReset, 0xB6)
            // TODO: delay 30ms
            //self.ctx = BMI088Context::default();
        }

        pub fn set_range(&mut self, range: Range) {
            self.write(Register::Range, range as u8);
            self.ctx.range = range;
        }

        pub fn set_bw(&mut self, bw: Bandwidth) {
            self.write(Register::Bandwidth, bw as u8);
            self.ctx.bandwidth = bw;
        }

        pub fn x(&self) -> f32 {
            let lsb = self.read(Register::RateXLsb) as u16;
            let msb = self.read(Register::RateXMsb) as u16;
            let range: f32 = self.ctx.range.into();

            let b = (msb * 256 + lsb) as i16;
            (b as f32 / 32767.0) * range
        }

        pub fn y(&self) -> f32 {
            let lsb = self.read(Register::RateYLsb) as u16;
            let msb = self.read(Register::RateYMsb) as u16;
            let range: f32 = self.ctx.range.into();

            let b = (msb * 256 + lsb) as i16;
            (b as f32 / 32767.0) * range
        }

        pub fn z(&self) -> f32 {
            let lsb = self.read(Register::RateZLsb) as u16;
            let msb = self.read(Register::RateZMsb) as u16;
            let range: f32 = self.ctx.range.into();

            let b = (msb * 256 + lsb) as i16;
            (b as f32 / 32767.0) * range
        }
    }

    pub struct Context {
        fifo_config_1: u8,
        fifo_config_0: u8,
        fifo_ext_int_s: u8,
        fifo_wm_en: u8,
        int3_int4_io_map: u8,
        int3_int4_io_conf: u8,
        int3_int4_io_ctrl: u8,
        lpm1: Lpm,
        bandwidth: Bandwidth,
        range: Range,
    }

    impl Context {
        pub fn default() -> Self {
            Self {
                fifo_config_1: 0x00,
                fifo_config_0: 0x00,
                fifo_ext_int_s: 0x00,
                fifo_wm_en: 0x00,
                int3_int4_io_map: 0x00,
                int3_int4_io_conf: 0x0F,
                int3_int4_io_ctrl: 0x00,
                lpm1: Lpm::Normal,
                bandwidth: Bandwidth::Odr2000Filter532,
                range: Range::_2000DegSec,
            }
        }
    }

    #[allow(dead_code)]
    enum Register {
        FifoData = 0x3F,
        FifoConfig1 = 0x3E,
        FifoConfig0 = 0x3D,
        SelfTest = 0x3C,
        FifoExtIntS = 0x34,
        FifoWmEnable = 0x1E,
        Int3Int4IoMap = 0x18,
        Int3Int4IoConf = 0x16,
        IntCtrl = 0x15,
        SoftReset = 0x14,
        Lpm = 0x11,
        Bandwidth = 0x10,
        Range = 0x0F,
        FifoStatus = 0x0E,
        IntStat = 0x0A,
        RateZMsb = 0x07,
        RateZLsb = 0x06,
        RateYMsb = 0x05,
        RateYLsb = 0x04,
        RateXMsb = 0x03,
        RateXLsb = 0x02,
        ChipId = 0x00,
    }

    #[derive(Clone, Copy)]
    pub enum Lpm {
        Suspend = 0x80,
        DeepSuspend = 0x20,
        Normal = 0x00,
    }

    #[derive(Clone, Copy)]
    pub enum Bandwidth {
        Odr100Filter32 = 0x07,
        Odr200Filter64 = 0x06,
        Odr100Filter12 = 0x05,
        Odr200Filter23 = 0x04,
        Odr400Filter47 = 0x03,
        Odr1000Filter116 = 0x02,
        Odr2000Filter230 = 0x01,
        Odr2000Filter532 = 0x00,
    }

    #[derive(Clone, Copy)]
    pub enum Range {
        _125DegSec = 0x04,
        _250DegSec = 0x03,
        _500DegSec = 0x02,
        _1000DegSec = 0x01,
        _2000DegSec = 0x00,
    }

    impl Into<f32> for Range {
        fn into(self) -> f32 {
            match self {
                Self::_125DegSec => 125.0,
                Self::_250DegSec => 250.0,
                Self::_500DegSec => 500.0,
                Self::_1000DegSec => 1000.0,
                Self::_2000DegSec => 2000.0,
            }
        }
    }
}
