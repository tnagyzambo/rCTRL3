use samv71_pac::{xdmac::xdmac_chid::XDMAC_CHID, XDMAC};

pub const DESC_VIEW0_DEFAULT: DescView0 = DescView0::default();

const DMA_CH_COUNT: usize = 24;

#[repr(C)]
pub struct DescView0 {
    nda: u32,
    ubc: u32,
    ta: u32,
}

impl DescView0 {
    pub const fn default() -> Self {
        Self {
            nda: 0,
            ubc: 0,
            ta: 0,
        }
    }

    pub fn new(nda: u32, ndaif: u32, ubc: u32, ta: u32) -> Self {
        // NDA must be maskable by 0xFFFFFFFC
        defmt::assert_eq!(nda, nda & 0xFFFFFFFC, "NDA memory alignment incorrect");

        // NDAIF must be maskable by 0x1
        defmt::assert_eq!(nda, nda & 0x1, "NDAIF memory alignment incorrect");

        // UBC must me maskable by 0xFFFFFF
        defmt::assert_eq!(ubc, ubc & 0xFFFFFF, "UBC memory alignment incorrect");

        Self {
            nda: nda | ndaif,
            ubc,
            ta,
        }
    }
}

/// DMA Channel.
pub struct Channel {
    pub ch: &'static XDMAC_CHID,
    id: usize,
}

impl Channel {
    /// Enable DMA channel.
    pub fn enable(&self) {
        let xdmac = unsafe { &*XDMAC::PTR };
        xdmac.ge().write(|w| unsafe { w.bits(1 << self.id) });
    }

    /// Force disable DMA channel.
    /// This function will block until the current transfer on the channel is complete
    /// and disable is sucessful.
    pub fn force_disable(&self) {
        let xdmac = unsafe { &*XDMAC::PTR };
        xdmac.gd().write(|w| unsafe { w.bits(1 << self.id) });

        // Wait until pending transfer is complete
        while xdmac.gs().read().bits() & (1 << self.id) != 0 {}
    }
}

pub struct Dma {
    xdmac: XDMAC,
}

impl Dma {
    pub fn new(xdmac: XDMAC) -> Self {
        Self { xdmac }
    }

    pub fn get_channel(&self) -> Option<Channel> {
        (0..DMA_CH_COUNT - 1).into_iter().find_map(|id| {
            let gs = self.xdmac.gs().read().bits();
            let mask = 1 << id;

            if gs & mask == 0 {
                // Have to reach into the void to conjure the XDMAC
                // so we can avoid borrowing our own XDMAC when getting
                // the channel reference
                let xdmac = unsafe { &*XDMAC::PTR };
                let ch = xdmac.xdmac_chid(id);

                // Clear pending interrupts
                let _ = ch.cis().read().bits();

                Some(Channel { ch, id })
            } else {
                None
            }
        })
    }
}
