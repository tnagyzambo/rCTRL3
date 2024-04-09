pub use crate::buffer::{
    RxBuf, RxBufData, RxBufDesc, RxFrame, TxBuf, TxBufData, TxBufDesc, TxFrame,
    RX_BUF_DATA_DEFAULT, RX_BUF_DESC_DEFAULT, TX_BUF_DATA_DEFAULT, TX_BUF_DESC_DEFAULT,
};
use core::marker::PhantomData;
use hal_macro::{BitFieldIterator, TryFromU8};
use samv71_pac::{Interrupt, GMAC, NVIC, PIOD, PMC};
use smoltcp::phy::{Device, DeviceCapabilities, Medium};
use smoltcp::time::Instant;

/// Maximum transmision unit of GMAC.
pub const MTU: usize = 1024;

/// Rx buffer count.
pub const RX_BUF_COUNT: usize = 16;

/// Tx buffer count.
pub const TX_BUF_COUNT: usize = 4; // Buffer count must be a multiple of 2

/// Dummy buffer count for unused priority queues.
pub const D_BUF_COUNT: usize = 2; // Buffer count must be a multiple of 2

#[repr(u8)]
#[derive(TryFromU8, BitFieldIterator)]
pub enum PhyInterrupt {
    Jabber = 7,
    RxError = 6,
    PageRx = 5,
    ParallelDetectFault = 4,
    LinkPartnerAcknowledge = 3,
    LinkDown = 2,
    RemoteFault = 1,
    LinkUp = 0,
}

#[repr(u8)]
#[derive(TryFromU8, BitFieldIterator)]
pub enum GmacInterrupt {
    TSUTIMCMP = 29,
    WOL = 28,
    RXLPISBC = 27,
    SRI = 26,
    PDRSFT = 25,
    PDRQFT = 24,
    PDRSFR = 23,
    PDRQFR = 22,
    SFT = 21,
    DRQFT = 20,
    SFR = 19,
    DRQFR = 18,
    PFTR = 14,
    PTZ = 13,
    PFNZ = 12,
    HRESP = 11,
    ROVR = 10,
    TCOMP = 7,
    TFC = 6,
    RLEX = 5,
    TUR = 4,
    TXUBR = 3,
    RXUBR = 2,
    RCOMP = 1,
    MFS = 0,
}

pub type MacEnabled = Mac<mac_state::Enabled>;

pub struct Mac<S: MacState> {
    pub gmac: GMAC,
    mac_addr: [u8; 6],
    tx_index: usize,
    rx_bufs: [RxBuf; RX_BUF_COUNT],
    tx_bufs: [TxBuf; TX_BUF_COUNT],
    drx_bufs: [RxBuf; D_BUF_COUNT],
    dtx_bufs: [TxBuf; D_BUF_COUNT],
    state: PhantomData<S>,
}

pub mod mac_state {
    pub struct Disabled {}
    pub struct Enabled {}
}

pub trait MacState {}
impl MacState for mac_state::Enabled {}
impl MacState for mac_state::Disabled {}

impl Mac<mac_state::Disabled> {
    pub fn new(
        gmac: GMAC,
        pmc: &PMC,
        piod: &PIOD,
        mac_addr: [u8; 6],
        rx_bufs: [RxBuf; RX_BUF_COUNT],
        tx_bufs: [TxBuf; TX_BUF_COUNT],
        drx_bufs: [RxBuf; D_BUF_COUNT],
        dtx_bufs: [TxBuf; D_BUF_COUNT],
    ) -> Self {
        // Enable GMAC interrupt in the NVIC
        unsafe {
            NVIC::unmask(Interrupt::GMAC);
            NVIC::unpend(Interrupt::GMAC);
        };

        // Enable peripheral clock to PIOD
        (*pmc).pcer0().write(|w| w.pid16().set_bit());

        // Set pin peripheral mode to A to give control of the pins
        // to the GMAC
        (*piod).abcdsr(0).modify(|_, w| {
            w.p0().clear_bit(); // GTXCK
            w.p1().clear_bit(); // GTXEN
            w.p2().clear_bit(); // GTX0
            w.p3().clear_bit(); // GTX1
            w.p4().clear_bit(); // GRXDV
            w.p5().clear_bit(); // GRX0
            w.p6().clear_bit(); // GRX1
            w.p7().clear_bit(); // GRXER
            w.p8().clear_bit(); // GMDC
            w.p9().clear_bit() // GMDIO
        });
        (*piod).abcdsr(1).modify(|_, w| {
            w.p0().clear_bit(); // GTXCK
            w.p1().clear_bit(); // GTXEN
            w.p2().clear_bit(); // GTX0
            w.p3().clear_bit(); // GTX1
            w.p4().clear_bit(); // GRXDV
            w.p5().clear_bit(); // GRX0
            w.p6().clear_bit(); // GRX1
            w.p7().clear_bit(); // GRXER
            w.p8().clear_bit(); // GMDC
            w.p9().clear_bit() // GMDIO
        });

        // Enable peripheral control of GMAC signal interface pins
        // The SAMV71 Xplained Ultra implements and RMII
        // connection, so only a subset of the connections found
        // in Table 37-1 are needed
        (*piod).pdr().write(|w| {
            w.p0().set_bit(); // GTXCK
            w.p1().set_bit(); // GTXEN
            w.p2().set_bit(); // GTX0
            w.p3().set_bit(); // GTX1
            w.p4().set_bit(); // GRXDV
            w.p5().set_bit(); // GRX0
            w.p6().set_bit(); // GRX1
            w.p7().set_bit(); // GRXER
            w.p8().set_bit(); // GMDC
            w.p9().set_bit() // GMDIO
        });

        // Enable peripheral clock to DMA and GMAC
        (*pmc).pcer1().write(|w| {
            w.pid39().set_bit(); // GMAC
            w.pid58().set_bit() // DMA
        });

        let mut mac = Self {
            gmac,
            mac_addr,
            tx_index: 0,
            rx_bufs,
            tx_bufs,
            drx_bufs,
            dtx_bufs,
            state: PhantomData,
        };

        // Disable GMAC Rx and Tx
        mac.gmac.ncr().modify(|_, w| {
            w.clrstat().set_bit(); // Clear statistics register
            w.rxen().clear_bit(); // Disable Rx
            w.txen().clear_bit() // Disable Tx
        });

        // Sanity check?
        // TODO: Not sure if needed
        if mac.mdio_busy() {
            defmt::warn!("PHY managment logic is busy durring MAC init");
        }

        // Set MAC address
        mac.gmac.gmac_sa(0).sab().write(|w| unsafe {
            let bottom: u32 = {
                ((mac_addr[3] as u32) << 24)
                    | ((mac_addr[2] as u32) << 16)
                    | ((mac_addr[1] as u32) << 8)
                    | (mac_addr[0] as u32)
            };
            w.addr().bits(bottom)
        });
        mac.gmac.gmac_sa(0).sat().write(|w| unsafe {
            let top: u16 = { ((mac_addr[5] as u16) << 8) | (mac_addr[4] as u16) };
            w.addr().bits(top)
        });

        // Set MDIO mode
        mac.gmac.ur().write(|w| {
            w.rmii().clear_bit() // RMII mode
        });
        mac.init_phy();

        // Set buffers
        mac.set_rx_bufs();
        mac.set_tx_bufs();
        mac.set_drx_bufs();
        mac.set_dtx_bufs();

        mac
    }

    // PHY init
    pub fn init_phy(&self) -> &Self {
        // Enable MIIM managment port
        self.gmac.ncr().modify(|_, w| w.mpe().set_bit());
        while self.mdio_busy() {}

        // Reset PHY
        self.mdio_write(0, 0x8000);

        // TODO: Fine grained control of interrupts
        // Enable PHY interrupt signals
        self.mdio_write(0x1B, 0x0500);

        // Clear pending interrupts
        _ = self.mdio_read(0x1B);

        self
    }

    /// Enable mulicast Rx from all adresses.
    pub fn enable_multicast_rx(&self) -> &Self {
        self.gmac
            .hrb()
            .write(|w| unsafe { w.addr().bits(0xFFFF_FFFF) });
        self.gmac
            .hrt()
            .write(|w| unsafe { w.addr().bits(0xFFFF_FFFF) });

        self
    }

    fn set_rx_bufs(&mut self) {
        // Set last buffer to wrap
        self.rx_bufs[RX_BUF_COUNT - 1].set_wrap();

        // Write location of RxBufDesc list to the GMAC_RBQB register
        self.gmac
            .rbqb()
            .write(|w| unsafe { w.bits(self.rx_bufs[0].desc.as_ptr().addr() as u32) });
    }

    fn set_tx_bufs(&mut self) {
        // Mark last descriptor in list
        self.tx_bufs[TX_BUF_COUNT - 1].set_wrap();

        // Write location of TxBufDescs to the GMAC_TBQB register
        self.gmac
            .tbqb()
            .write(|w| unsafe { w.bits(self.tx_bufs[0].desc.as_ptr().addr() as u32) });
    }

    fn set_drx_bufs(&mut self) {
        // Set last buffer to wrap
        self.drx_bufs[D_BUF_COUNT - 1].set_wrap();

        // Write location for dummy RxBufDesc to GMAC_RBQBAPQ registers
        for buf in self.gmac.rbqbapq_iter() {
            buf.write(|w| unsafe { w.bits(self.drx_bufs[0].desc.as_ptr().addr() as u32) })
        }
    }

    fn set_dtx_bufs(&mut self) {
        // Set last buffer to wrap
        self.dtx_bufs[D_BUF_COUNT - 1].set_wrap();

        // Write location for unused TxBufDesc to GMAC_TBQBAPQ registers
        for buf in self.gmac.tbqbapq_iter() {
            buf.write(|w| unsafe { w.bits(self.dtx_bufs[0].desc.as_ptr().addr() as u32) })
        }
    }

    // TODO: Make sane
    pub fn config_network_default(&self) -> &Self {
        // Set network configuration
        self.gmac.ncfgr().write(|w| {
            w.rxcoen().set_bit(); // Enable checksum offload
            unsafe { w.dbw().bits(0) }; // 32 bit data bus width
            w.clk().mck_64(); // MCK divided by 64
            w.rfcs().clear_bit(); // Retain frame check sequence
            w.fd().set_bit(); // Enable full duplex
            w.spd().set_bit() // 100 Mbps
        });

        self
    }

    // TODO: Make sane
    pub fn config_dma_default(&self) -> &Self {
        let drbs = (MTU / 64).min(255) as u8;
        defmt::assert_ne!(drbs, 0, "Invalid MTU");

        // Set up DMA configuration register
        self.gmac.dcfgr().write(|w| {
            w.ddrp().set_bit(); // Discard Rx packets when no AHB resources are available
            unsafe {
                w.drbs().bits(drbs);
            }
            w.txcoen().set_bit(); // Enable checksum offload
            w.txpbms().set_bit(); // Use full 4KBytes of Tx packet buffer size
            w.rxbms().full(); // Use full 4KBytes of Rx packet buffer size
            w.espa().clear_bit(); // Little endian mode for packet data access
            w.esma().clear_bit(); // little endian mode for managment descriptor access
            w.fbldo().incr4() // Select data burst lenght of 4
        });

        self
    }

    /// Enable GMAC Rx and Tx.
    pub fn enable(self) -> Mac<mac_state::Enabled> {
        // Enable GMAC Rx and Tx
        self.gmac.ncr().modify(|_, w| {
            w.clrstat().set_bit(); // Clear statistics register
            w.rxen().set_bit(); // Enable Rx
            w.txen().set_bit() // Enable Tx
        });

        // TODO: Fine grained control over interrupts
        self.gmac.ier().write(|w| unsafe { w.bits(0xFFFF) });

        Mac {
            gmac: self.gmac,
            mac_addr: self.mac_addr,
            tx_index: self.tx_index,
            rx_bufs: self.rx_bufs,
            tx_bufs: self.tx_bufs,
            drx_bufs: self.drx_bufs,
            dtx_bufs: self.dtx_bufs,
            state: PhantomData,
        }
    }
}

impl Mac<mac_state::Enabled> {
    /// Get a complete RxFrame from the buffers.
    pub fn get_rx_frame<'a>(rx_bufs: &'a mut [RxBuf]) -> Option<RxFrame<'a>> {
        // Scan for full RxBuf
        rx_bufs.iter_mut().find_map(|buf| {
            if buf.owned() {
                Some(RxFrame::new(buf))
            } else {
                None
            }
        })
    }

    /// Get the next empty buffer to write a TxFrame to.
    pub fn get_tx_frame<'b>(tx_bufs: &'b mut [TxBuf], index: &'b mut usize) -> Option<TxFrame<'b>> {
        // Scan for free TxBuf
        // When the GMAC receives TSTART it does not scan from first entry in the buffer list
        // Instead it looks for frame to send at the next entry in the list after the previous send
        // Therefor you cannot find the first free descriptor in the list as it will fail to send
        // until for whatever reason the GMAC wraps back around to it.

        let buf = &mut tx_bufs[*index];
        if buf.owned() {
            Some(TxFrame::new(buf, index))
        } else {
            None
        }
    }

    /// Disable GMAC Rx and Tx.
    pub fn disable(self) -> Mac<mac_state::Disabled> {
        // Disable GMAC Rx and Tx
        self.gmac.ncr().modify(|_, w| {
            w.clrstat().set_bit(); // Clear statistics register
            w.rxen().clear_bit(); // Disable Rx
            w.txen().clear_bit() // Disable Tx
        });

        // Disable GMAC interrupts
        self.gmac.idr().write(|w| unsafe { w.bits(0xFFFFFFFF) });

        // Clear GMAC interrupts by reading
        let _ = self.gmac.isr().read().bits();

        // Disable interrupts in all five GMAC queues
        for idrpq in self.gmac.idrpq_iter() {
            idrpq.write(|w| unsafe { w.bits(0xFFFFFFFF) });
        }

        // Clear interrupts in all five GMAC queues by reading
        for isrpq in self.gmac.isrpq_iter() {
            let _ = isrpq.read().bits();
        }

        // Clear GMAC Rx status register by writing
        self.gmac.rsr().write(|w| unsafe { w.bits(0x0F) });

        // Clear GMAC Tx status register by writing
        self.gmac.tsr().write(|w| unsafe { w.bits(0x1FF) });

        Mac {
            gmac: self.gmac,
            mac_addr: self.mac_addr,
            tx_index: self.tx_index,
            rx_bufs: self.rx_bufs,
            tx_bufs: self.tx_bufs,
            drx_bufs: self.drx_bufs,
            dtx_bufs: self.dtx_bufs,
            state: PhantomData,
        }
    }
}

impl<S: MacState> Mac<S> {
    pub fn mac_addr(&self) -> [u8; 6] {
        self.mac_addr
    }

    fn mdio_write(&self, reg_idx: u8, op_data: u16) {
        // Perform a MDIO write to a clause 22 PHY
        self.gmac.man().write(|w| {
            w.wzo().clear_bit(); // Mandatory 0 feild
            w.cltto().set_bit(); // Clause 22 PHY operation
            unsafe {
                w.op().bits(0b01); // Write operation
                w.wtn().bits(0b10); // Mandatory 10 feild
                w.phya().bits(0);
                w.rega().bits(reg_idx);
                w.data().bits(op_data)
            }
        });

        while self.mdio_busy() {}
    }

    pub fn mdio_read(&self, reg_idx: u8) -> u16 {
        self.gmac.man().write(|w| {
            w.wzo().clear_bit(); // Mandatory 0 feild
            w.cltto().set_bit(); // Clause 22 PHY operation
            unsafe {
                w.op().bits(0b10); // Read operation
                w.wtn().bits(0b10); // Mandatory 10 feild
                w.phya().bits(0);
                w.rega().bits(reg_idx);
                w.data().bits(0)
            }
        });

        // Block until read operation has completed
        while self.mdio_busy() {}

        self.gmac.man().read().data().bits()
    }

    // PHY managment logic idle
    fn mdio_busy(&self) -> bool {
        // GMAC_NSR bit 2 is set when PHY is idle
        self.gmac.nsr().read().idle().bit_is_clear()
    }

    pub fn mdio_interrupts(&self) -> PhyInterruptIterator {
        PhyInterruptIterator::new(self.mdio_read(0x1B) as u32)
    }

    pub fn interrupts(&self) -> GmacInterruptIterator {
        GmacInterruptIterator::new(self.gmac.isr().read().bits())
    }
}

impl Device for Mac<mac_state::Enabled> {
    type RxToken<'a> = RxFrame<'a> where Self: 'a;
    type TxToken<'a> = TxFrame<'a> where Self: 'a;

    fn receive(&mut self, _timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let rx_frame = Self::get_rx_frame(&mut self.rx_bufs[..])?;
        let tx_frame = Self::get_tx_frame(&mut self.tx_bufs[..], &mut self.tx_index)?;

        Some((rx_frame, tx_frame))
    }

    fn transmit(&mut self, _timestap: Instant) -> Option<Self::TxToken<'_>> {
        Self::get_tx_frame(&mut self.tx_bufs[..], &mut self.tx_index)
    }

    // TODO: Figure out what needs to go here
    fn capabilities(&self) -> DeviceCapabilities {
        let mut cap = DeviceCapabilities::default();
        cap.max_transmission_unit = 1518;
        cap.max_burst_size = Some(4);
        cap.medium = Medium::Ethernet;
        cap
    }
}
