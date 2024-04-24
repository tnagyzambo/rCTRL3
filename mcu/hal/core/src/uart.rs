use crate::clock::mck::{MckDiv, MckPres, MckSource};
use crate::clock::{Clock, Mck};
use crate::pio::*;
use core::marker::PhantomData;
use hal_macro::{BitFieldIterator, TryFromU8};
use samv71_pac::uart0::RegisterBlock;
use samv71_pac::{Interrupt, NVIC, PMC, UART0, UART1, UART2, UART3, UART4};

#[repr(u8)]
#[derive(TryFromU8, BitFieldIterator)]
pub enum UartInterrupt {
    CMP = 15,
    TXEMTPY = 9,
    PARE = 7,
    FRAME = 6,
    OVRE = 5,
    TXRDY = 1,
    RXRDY = 0,
}

pub type Br = u16;

pub struct Uart<U: UartId> {
    uart: PhantomData<U>,
    _br: Br,
    _rx: U::RX,
    _tx: U::TX,
}

impl<U: UartId> Uart<U> {
    /// Initialize a UART peripheral.
    pub fn init<C: Clock + UartClockSource>(rx: U::RX, tx: U::TX, _clk: C, br: Br) -> Self {
        // Enable clock to UART peripheral
        Self::enable_peripheral();

        let uart = unsafe { &*U::REG };

        // Select baud rate generactor clock source
        uart.mr().modify(|_, w| w.brsrcck().bit(C::BRSRCCK));

        // Calculate baud rate generator clock divisor value and set it
        let cd = C::FREQ as u16 / (16 * br);
        uart.brgr().write(|w| unsafe { w.cd().bits(cd) });

        uart.cr().write(|w| {
            w.rxen().set_bit(); // Enable Rx
            w.txen().set_bit() // Enable Tx
        });

        uart.mr().modify(|_, w| w.chmode().local_loopback());

        Self {
            uart: PhantomData,
            _br: br,
            _rx: rx,
            _tx: tx,
        }
    }

    fn enable_peripheral() {
        // Enable clock to TWIHS peripheral
        let pmc = unsafe { &*PMC::PTR };

        if U::PER <= 31 {
            pmc.pcer0().write(|w| unsafe { w.bits(1 << U::PER) });
        } else {
            pmc.pcer1().write(|w| unsafe { w.bits(1 << (U::PER - 32)) });
        }
    }

    pub fn enable_interrupts(&self) {
        unsafe { NVIC::unmask(U::INT) }; // Enable uart interrupts in the NVIC
        NVIC::unpend(U::INT); // Unpend uart interrupts

        let uart = unsafe { &*U::REG };
        let _ = uart.sr().read().bits(); // Clear interrupt status register

        uart.ier().write(|w| {
            //w.cmp().set_bit();
            //w.txempty().set_bit();
            w.pare().set_bit();
            w.frame().set_bit();
            w.ovre().set_bit()
            //w.txrdy().set_bit();
            //w.rxrdy().set_bit()
        });
    }

    pub fn interrupts(&self) -> UartInterruptIterator {
        let uart = unsafe { &*U::REG };
        UartInterruptIterator::new(uart.sr().read().bits())
    }

    pub fn read(&self) -> u8 {
        let uart = unsafe { &*U::REG };
        uart.rhr().read().bits() as u8
    }

    pub fn write(&self, data: u8) {
        let uart = unsafe { &*U::REG };
        uart.thr().write(|w| unsafe { w.txchr().bits(data) });
    }
}

/// UartId is implement by valid UART instances
pub trait UartId {
    const REG: *const RegisterBlock;
    const INT: Interrupt;
    const PER: u8;
    type RX;
    type TX;
}

pub struct Uart0 {}

impl UartId for Uart0 {
    const REG: *const RegisterBlock = UART0::PTR;
    const INT: Interrupt = Interrupt::UART0;
    const PER: u8 = 7;
    type RX = Pin<PA9, PeripheralA>;
    type TX = Pin<PA10, PeripheralA>;
}

pub struct Uart1 {}

impl UartId for Uart1 {
    const REG: *const RegisterBlock = UART1::PTR;
    const INT: Interrupt = Interrupt::UART1;
    const PER: u8 = 8;
    type RX = Pin<PA5, PeripheralC>;
    type TX = Pin<PA4, PeripheralC>;
}

pub struct Uart2 {}

impl UartId for Uart2 {
    const REG: *const RegisterBlock = UART2::PTR;
    const INT: Interrupt = Interrupt::UART2;
    const PER: u8 = 44;
    type RX = Pin<PD25, PeripheralC>;
    type TX = Pin<PD26, PeripheralC>;
}

pub struct Uart3 {}

impl UartId for Uart3 {
    const REG: *const RegisterBlock = UART3::PTR;
    const INT: Interrupt = Interrupt::UART3;
    const PER: u8 = 45;
    type RX = Pin<PD28, PeripheralA>;
    type TX = Pin<PD30, PeripheralA>;
}

pub struct Uart4 {}

impl UartId for Uart4 {
    const REG: *const RegisterBlock = UART4::PTR;
    const INT: Interrupt = Interrupt::UART4;
    const PER: u8 = 46;
    type RX = Pin<PD18, PeripheralC>;
    type TX = Pin<PD19, PeripheralC>;
}

/// UartClockSource is implemented by valid UART clock sources.
pub trait UartClockSource {
    const BRSRCCK: bool;
}

/// MCK
impl<S: Clock + MckSource, P: MckPres, D: MckDiv> UartClockSource for Mck<S, P, D> {
    const BRSRCCK: bool = false;
}

//
//
//const MTU: usize = 128;
//
//type RxBufDesc = DescView0;
//type RxBufData = [u8; MTU];
//
//const RX_BUF_DESC_DEFAULT: RxBufDesc = DESC_VIEW0_DEFAULT;
//const RX_BUF_DATA_DEFAULT: RxBufData = [0; MTU];
//
//pub struct RxBuf {
//    desc: NonNull<RxBufDesc>,
//    data: NonNull<RxBufData>,
//}
//
//impl RxBuf {
//    fn new(desc: &mut RxBufDesc, data: &mut RxBufData) -> Self {
//        let desc = NonNull::from(desc);
//        let data = NonNull::from(data);
//
//        Self { desc, data }
//    }
//}
//
//type TxBufData = [u8; MTU];
//
//const TX_BUF_DATA_DEFAULT: TxBufData = [0; MTU];
//
//pub struct TxBuf {
//    data: NonNull<RxBufData>,
//}
//
//impl TxBuf {
//    fn new(data: &mut RxBufData) -> Self {
//        let data = NonNull::from(data);
//
//        Self { data }
//    }
//}
//
//pub struct Storage<const N_RX: usize, const N_TX: usize> {
//    rx_descs: [RxBufDesc; N_RX],
//    rx_datas: [RxBufData; N_RX],
//    tx_datas: [TxBufData; N_TX],
//}
//
//impl<const N_RX: usize, const N_TX: usize> Storage<N_RX, N_TX> {
//    pub const fn default() -> Self {
//        Self {
//            rx_descs: [RX_BUF_DESC_DEFAULT; N_RX],
//            rx_datas: [RX_BUF_DATA_DEFAULT; N_RX],
//            tx_datas: [TX_BUF_DATA_DEFAULT; N_TX],
//        }
//    }
//}
//
//pub struct StorageMap<const N_RX: usize, const N_TX: usize> {
//    rx_bufs: [RxBuf; N_RX],
//    tx_bufs: [TxBuf; N_TX],
//}
//
//impl<const N_RX: usize, const N_TX: usize> From<&mut Storage<N_RX, N_TX>>
//    for StorageMap<N_RX, N_TX>
//{
//    fn from(storage: &mut Storage<N_RX, N_TX>) -> Self {
//        let rx_bufs: [RxBuf; N_RX] = core::array::from_fn(|i| {
//            RxBuf::new(&mut storage.rx_descs[i], &mut storage.rx_datas[i])
//        });
//
//        let tx_bufs: [TxBuf; N_TX] = core::array::from_fn(|i| TxBuf::new(&mut storage.tx_datas[i]));
//
//        Self { rx_bufs, tx_bufs }
//    }
//}
//
//// DMA Rx channel
//let dma_rx = dma.get_channel().unwrap();

//// Point DMA Rx channel to first desc in list
//let rx_buf_desc_addr = storage_map.rx_bufs[0].desc.as_ptr().addr() as u32;
//dma_rx.ch.cnda().write(|w| {
//    unsafe { w.nda().bits(rx_buf_desc_addr) }; // Set next descriptor address to first buffer desc
//    w.ndaif().clear_bit() // Use bus 0 for desc loads?
//});

//// Configure DMA Rx channel descriptor control
//dma_rx.ch.cndc().write(|w| {
//    w.ndview().ndv0(); // Set next descriptor view to type 0
//    w.nddup().dst_params_updated(); // Change destination params on descriptor load
//    w.ndsup().src_params_updated(); // Change source params on descriptor load
//    w.nde().dscr_fetch_en() // Enable descriptor fetch
//});

//// Configure DMA Rx channel interrupts
//// NOTE: The UART Rx channel is a ring buffer so we should never see the LIE interrupt
//dma_rx.ch.cie().write(|w| {
//    w.lie().set_bit() // Set end of linked list interrupt
//});

//// Configure DMA Rx channel
//dma_rx.ch.cc().write(|w| {
//    w.perid().usart0_rx(); // TODO: configurable?
//    w.dam().incremented_am(); // ???
//    w.dif().ahb_if0(); // ???
//    w.sif().ahb_if0(); // ???
//    w.dwidth().byte(); // UART uses 8 bit words
//    w.csize().chk_1(); // UART transfers data 1 bit at a time
//    w.swreq().hwr_connected(); // HW controlled transfer
//    w.dsync().per2mem(); // Peripheral to memory transfer
//    w.mbsize().eight(); // Memory burst size 8bits
//    w.type_().per_tran() // Peripheral transfer
//});

// Enable DMA Rx channel
//dma_rx.enable();

// DMA Tx channel
//let dma_tx = dma.get_channel().unwrap();

//dma_tx.ch.csa().write(|w| w);

//dma_tx.ch.cda().write(|w| w);

//dma_tx.ch.cc().write(|w| w);

//dma_tx.ch.cndc().write(|w| unsafe { w.bits(0x0) });
//dma_tx.ch.cds_msp().write(|w| unsafe { w.bits(0x0) });
//dma_tx.ch.csus().write(|w| unsafe { w.bits(0x0) });
//dma_tx.ch.cdus().write(|w| unsafe { w.bits(0x0) });

//// Configure DMA Tx channel interrupts
//dma_rx.ch.cie().write(|w| {
//    w.bie().set_bit() // Set end of linked list interrupt
//});
