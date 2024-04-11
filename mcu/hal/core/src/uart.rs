use crate::dma::{Channel, DescView0, Dma, DESC_VIEW0_DEFAULT};
use core::{marker::PhantomData, ptr::NonNull};
use hal_macro::{BitFieldIterator, TryFromU8};
use samv71_pac::{UART0, UART1, UART2, UART3, UART4};

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

const MTU: usize = 128;

type RxBufDesc = DescView0;
type RxBufData = [u8; MTU];

const RX_BUF_DESC_DEFAULT: RxBufDesc = DESC_VIEW0_DEFAULT;
const RX_BUF_DATA_DEFAULT: RxBufData = [0; MTU];

pub struct RxBuf {
    desc: NonNull<RxBufDesc>,
    data: NonNull<RxBufData>,
}

impl RxBuf {
    fn new(desc: &mut RxBufDesc, data: &mut RxBufData) -> Self {
        let desc = NonNull::from(desc);
        let data = NonNull::from(data);

        Self { desc, data }
    }
}

type TxBufData = [u8; MTU];

const TX_BUF_DATA_DEFAULT: TxBufData = [0; MTU];

pub struct TxBuf {
    data: NonNull<RxBufData>,
}

impl TxBuf {
    fn new(data: &mut RxBufData) -> Self {
        let data = NonNull::from(data);

        Self { data }
    }
}

pub struct Storage<const N_RX: usize, const N_TX: usize> {
    rx_descs: [RxBufDesc; N_RX],
    rx_datas: [RxBufData; N_RX],
    tx_datas: [TxBufData; N_TX],
}

impl<const N_RX: usize, const N_TX: usize> Storage<N_RX, N_TX> {
    pub const fn default() -> Self {
        Self {
            rx_descs: [RX_BUF_DESC_DEFAULT; N_RX],
            rx_datas: [RX_BUF_DATA_DEFAULT; N_RX],
            tx_datas: [TX_BUF_DATA_DEFAULT; N_TX],
        }
    }
}

pub struct StorageMap<const N_RX: usize, const N_TX: usize> {
    rx_bufs: [RxBuf; N_RX],
    tx_bufs: [TxBuf; N_TX],
}

impl<const N_RX: usize, const N_TX: usize> From<&mut Storage<N_RX, N_TX>>
    for StorageMap<N_RX, N_TX>
{
    fn from(storage: &mut Storage<N_RX, N_TX>) -> Self {
        let rx_bufs: [RxBuf; N_RX] = core::array::from_fn(|i| {
            RxBuf::new(&mut storage.rx_descs[i], &mut storage.rx_datas[i])
        });

        let tx_bufs: [TxBuf; N_TX] = core::array::from_fn(|i| TxBuf::new(&mut storage.tx_datas[i]));

        Self { rx_bufs, tx_bufs }
    }
}

pub type Uart0<const N_RX: usize, const N_TX: usize> = Uart<UART0, N_RX, N_TX>;

pub struct Uart<U, const N_RX: usize, const N_TX: usize> {
    uart: UART2,
    dma_rx: Channel,
    dma_tx: Channel,
    storage_map: StorageMap<N_RX, N_TX>,
    delete_me: PhantomData<U>,
}

// TODO: Remove this
unsafe impl<U, const N_RX: usize, const N_TX: usize> Send for Uart<U, N_RX, N_TX> {}

impl<U, const N_RX: usize, const N_TX: usize> Uart<U, N_RX, N_TX> {
    pub fn new(uart: UART2, storage: &'static mut Storage<N_RX, N_TX>, dma: &Dma) -> Self {
        // Map storage
        let storage_map = StorageMap::from(storage);

        // Config UART
        uart.cr().write(|w| {
            w.rxen().set_bit();
            w.txen().set_bit()
        });

        uart.brgr().write(|w| unsafe { w.bits(10000) });

        uart.mr().modify(|_, w| w.chmode().local_loopback());

        uart.ier().write(|w| {
            //w.cmp().set_bit();
            //w.txempty().set_bit();
            w.pare().set_bit();
            w.frame().set_bit();
            w.ovre().set_bit()
            //w.txrdy().set_bit();
            //w.rxrdy().set_bit()
        });

        // DMA Rx channel
        let dma_rx = dma.get_channel().unwrap();

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
        let dma_tx = dma.get_channel().unwrap();

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

        Self {
            uart,
            dma_rx,
            dma_tx,
            storage_map,
            delete_me: PhantomData,
        }
    }

    pub fn interrupts(&self) -> UartInterruptIterator {
        UartInterruptIterator::new(self.uart.sr().read().bits())
    }

    pub fn get_write_buffer(&mut self) -> &mut TxBufData {
        unsafe { self.storage_map.tx_bufs[0].data.as_mut() }
    }

    //pub fn write(&self, len: u16) {
    //    self.dma_tx
    //        .ch
    //        .cbc()
    //        .modify(|_, w| unsafe { w.blen().bits(len) });

    //    self.dma_tx.enable();
    //}

    pub fn read(&self) -> u8 {
        self.uart.rhr().read().bits() as u8
    }

    pub fn write(&self, data: u8) {
        self.uart.thr().write(|w| unsafe { w.txchr().bits(data) });
    }
}
