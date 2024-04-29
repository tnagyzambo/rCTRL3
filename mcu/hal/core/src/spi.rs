use crate::pio::{
    PeripheralB, PeripheralC, PeripheralD, Pin, PinPeripheral, PB2, PC24, PC25, PC26, PC27, PC28,
    PC29, PC30, PD12, PD20, PD21, PD22, PD25, PD27,
};
use core::marker::PhantomData;
use hal_macro::{BitFieldIterator, TryFromU8};
use samv71_pac::spi0::RegisterBlock;
use samv71_pac::{Interrupt, NVIC, PMC, SPI0, SPI1};

//
//
//

pub trait SpiDevice<C: ChipSelect> {
    fn read(&self) -> u16;
    fn write(&self, data: u16);
}

#[repr(u8)]
#[derive(TryFromU8, BitFieldIterator)]
pub enum SpiInterrupt {
    UNDES = 10,
    TXEMPTY = 9,
    NSSR = 8,
    OVRES = 3,
    MODF = 2,
    TDRE = 1,
    RDRF = 0,
}

//
//
//

pub trait SpiMode {}

pub struct Client {}
impl SpiMode for Client {}

pub struct HostFixed {}
impl SpiMode for HostFixed {}

pub struct HostVar {}
impl SpiMode for HostVar {}

pub struct HostDecode {}
impl SpiMode for HostDecode {}

//
//
//

pub struct Spi<I: SpiId, M: SpiMode> {
    spi: PhantomData<I>,
    mode: PhantomData<M>,
    _spck: I::SPCK,
    _miso: I::MISO,
    _mosi: I::MOSI,
}

impl<I: SpiId> Spi<I, HostVar> {
    pub fn init(spck: I::SPCK, miso: I::MISO, mosi: I::MOSI) -> Self {
        // Enable clock to SPI peripheral
        Self::enable_peripheral();

        let spi = unsafe { &*I::REG };

        spi.cr().write(|w| w.spien().set_bit());

        spi.mr().modify(|_, w| {
            w.mstr().master(); // Set SPI to host mode
            w.pcsdec().clear_bit();
            w.modfdis().set_bit();
            w.ps().set_bit() // Variable peripheral selection
        });

        Self {
            spi: PhantomData,
            mode: PhantomData,
            _spck: spck,
            _miso: miso,
            _mosi: mosi,
        }
    }

    pub fn cs0(&self) -> &impl SpiDevice<CS<0>> {
        self
    }

    pub fn cs1(&self) -> &impl SpiDevice<CS<1>> {
        self
    }

    pub fn cs2(&self) -> &impl SpiDevice<CS<2>> {
        self
    }
}

impl<I: SpiId, C: ChipSelect> SpiDevice<C> for Spi<I, HostVar> {
    fn write(&self, data: u16) {
        let spi = unsafe { &*I::REG };

        // Spin until Tx ready
        while spi.sr().read().tdre().bit_is_clear() {}

        spi.tdr().write(|w| {
            //w.lastxfer().set_bit();
            unsafe { w.pcs().bits(C::PCS) }; // Chip select
            unsafe { w.td().bits(data) } // Tx data
        });
    }

    fn read(&self) -> u16 {
        let spi = unsafe { &*I::REG };

        // Spin until Rx ready
        while spi.sr().read().rdrf().bit_is_clear() {}

        spi.rdr().read().rd().bits()
    }
}

//
//
//

impl<I: SpiId, M: SpiMode> Spi<I, M> {
    fn enable_peripheral() {
        // Enable clock to I2C peripheral
        let pmc = unsafe { &*PMC::PTR };
        if I::PER <= 31 {
            pmc.pcer0().write(|w| unsafe { w.bits(1 << I::PER) });
        } else {
            pmc.pcer1().write(|w| unsafe { w.bits(1 << (I::PER - 32)) });
        }
    }

    pub fn enable_interrupts(&self) {
        unsafe { NVIC::unmask(I::INT) }; // Enable spi interrupts in the NVIC
        NVIC::unpend(I::INT); // Unpend spi interrupts

        let spi = unsafe { &*I::REG };
        let _ = spi.sr().read().bits(); // Clear interrupt status register

        spi.ier().write(|w| {
            //w.rdrf().set_bit();
            //w.tdre().set_bit();
            w.modf().set_bit();
            w.ovres().set_bit();
            w.nssr().set_bit();
            //w.txempty().set_bit();
            w.undes().set_bit()
        })
    }

    pub fn interrupts(&self) -> SpiInterruptIterator {
        let spi = unsafe { &*I::REG };
        SpiInterruptIterator::new(spi.sr().read().bits())
    }
}

//
//
//

pub trait Val {
    const VAL: usize;
}

pub trait ChipSelect: Val {
    const V: usize = Self::VAL;
    const PCS: u8;
}

pub struct CS<const N: usize>;

impl<const N: usize> Val for CS<N> {
    const VAL: usize = N;
}

impl ChipSelect for CS<0> {
    const PCS: u8 = 0b0000;
}

impl ChipSelect for CS<1> {
    const PCS: u8 = 0b0001;
}

impl ChipSelect for CS<2> {
    const PCS: u8 = 0b0011;
}

impl ChipSelect for CS<3> {
    const PCS: u8 = 0b0111;
}

//
//
//

pub trait SpiId {
    const REG: *const RegisterBlock;
    const INT: Interrupt;
    const PER: u8;
    type SPCK: PinPeripheral;
    type MISO: PinPeripheral;
    type MOSI: PinPeripheral;
    type CS0: PinPeripheral;
    type CS1: PinPeripheral;
    type CS2: PinPeripheral;
    type CS3: PinPeripheral;
}

pub struct Spi0;

impl SpiId for Spi0 {
    const REG: *const RegisterBlock = SPI0::PTR;
    const INT: Interrupt = Interrupt::SPI0;
    const PER: u8 = 21;
    type SPCK = Pin<PD22, PeripheralB>;
    type MISO = Pin<PD20, PeripheralB>;
    type MOSI = Pin<PD21, PeripheralB>;
    type CS0 = Pin<PB2, PeripheralD>;
    type CS1 = Pin<PD25, PeripheralB>;
    type CS2 = Pin<PD12, PeripheralC>;
    type CS3 = Pin<PD27, PeripheralB>;
}

pub struct Spi1;

impl SpiId for Spi1 {
    const REG: *const RegisterBlock = SPI1::PTR;
    const INT: Interrupt = Interrupt::SPI1;
    const PER: u8 = 42;
    type SPCK = Pin<PC24, PeripheralC>;
    type MISO = Pin<PC26, PeripheralC>;
    type MOSI = Pin<PC27, PeripheralC>;
    type CS0 = Pin<PC25, PeripheralC>;
    type CS1 = Pin<PC28, PeripheralC>;
    type CS2 = Pin<PC29, PeripheralC>;
    type CS3 = Pin<PC30, PeripheralC>;
}
