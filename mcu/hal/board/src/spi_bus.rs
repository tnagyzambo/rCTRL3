use crate::bmi088;
use hal_core::pio::PinPeripheral;
use hal_core::spi::*;

pub struct SpiBus<I: SpiId> {
    pub spi: Spi<I, HostVar>,
    ctx_gyro: bmi088::gyro::Context,
    ctx_accl: bmi088::accl::Context,
}

impl<I: SpiId> SpiBus<I> {
    pub fn init() -> Self {
        // SPI
        let spck = <I>::SPCK::init();
        let miso = <I>::MISO::init();
        let mosi = <I>::MOSI::init();
        let _cs0 = <I>::CS0::init();
        let _cs1 = <I>::CS1::init();
        let _cs2 = <I>::CS2::init();
        let _cs3 = <I>::CS3::init();
        let spi = Spi::<I, HostVar>::init(spck, miso, mosi);
        spi.enable_interrupts();

        // Hack config of SPI bus here
        // TODO: Make this better
        let spi_reg = unsafe { &*I::REG };

        spi_reg.csr(0).write(|w| {
            unsafe { w.dlybs().bits(0x10) };
            unsafe { w.dlybct().bits(0x00) };
            unsafe { w.scbr().bits(250) }; // Serial Clock Bit Rate
                                           //w.cpol().idle_high();
            w.ncpha().valid_leading_edge();
            w.bits_()._16_bit(); // Bits per transfer
            w.csaat().set_bit()
        });

        spi_reg.csr(1).write(|w| {
            unsafe { w.dlybs().bits(0x10) };
            unsafe { w.dlybct().bits(0x00) };
            unsafe { w.scbr().bits(250) }; // Serial Clock Bit Rate
                                           //w.cpol().idle_high();
            w.ncpha().valid_leading_edge();
            w.bits_()._16_bit(); // Bits per transfer
            w.csaat().set_bit()
        });

        // IMU
        let ctx_gyro = bmi088::gyro::Context::default();
        let ctx_accl = bmi088::accl::Context::default();

        Self {
            spi,
            ctx_gyro,
            ctx_accl,
        }
    }

    pub fn gyro(&mut self) -> bmi088::Gyro {
        bmi088::Gyro::from(&mut self.ctx_gyro, self.spi.cs1())
    }

    pub fn accelerometer(&mut self) -> bmi088::Accelerometer {
        bmi088::Accelerometer::from(&mut self.ctx_accl, self.spi.cs0())
    }
}
