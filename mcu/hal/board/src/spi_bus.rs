use crate::bmi088::*;
use hal_core::pio::PinPeripheral;
use hal_core::spi::*;

pub struct SpiBus<I: SpiId> {
    pub spi: Spi<I, HostVar>,
    imu_state: BMI088Config,
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

        spi_reg.csr(1).write(|w| {
            unsafe { w.dlybs().bits(0x10) };
            unsafe { w.dlybct().bits(0x20) };
            unsafe { w.scbr().bits(250) }; // Serial Clock Bit Rate
                                           //w.cpol().idle_high();
                                           //w.ncpha().valid_leading_edge();
            w.bits_()._16_bit(); // Bits per transfer
            w.csaat().clear_bit()
        });

        // IMU
        let imu_state = BMI088Config::default();

        Self { spi, imu_state }
    }

    pub fn imu(&mut self) -> &impl BMI088<CS<1>> {
        self.spi.cs1()
    }
}
