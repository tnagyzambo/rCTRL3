use crate::gmac::{MTU, TX_BUF_COUNT};
use core::{
    ops::FnOnce,
    ptr::NonNull,
    sync::atomic::{compiler_fence, Ordering},
};
use pac::GMAC;
use samv71_pac as pac;
use smoltcp::phy::{RxToken, TxToken};

/// Default zero initialized Rx buffer descriptor.
pub const RX_BUF_DESC_DEFAULT: RxBufDesc = [0; 2];

/// Default zero initialized Rx buffer.
pub const RX_BUF_DATA_DEFAULT: RxBufData = [0; MTU];

pub type RxBufDesc = [u32; 2];
pub type RxBufData = [u8; MTU];

pub struct RxBuf {
    pub desc: NonNull<RxBufDesc>,
    pub data: NonNull<RxBufData>,
}

impl RxBuf {
    // TODO: Return error instead of assert
    pub fn new(desc: &mut RxBufDesc, data: &mut RxBufData) -> Self {
        let mut desc = NonNull::from(desc);
        let data = NonNull::from(data);

        // RxBufDesc must have an address maskable by 0xFFFFFFFC
        let desc_addr = desc.as_ptr().addr() as u32;
        defmt::assert_eq!(
            desc_addr,
            desc_addr & 0xFFFFFFFC,
            "RxBufDesc memory alignment incorrect"
        );

        // RxBufData must have an address maskable by 0xFFFFFFFC
        let data_addr = data.as_ptr().addr() as u32;
        defmt::assert_eq!(
            data_addr,
            data_addr & 0xFFFFFFFC,
            "RxBufData memory alignment incorrect"
        );

        // Sets the value of the `addr` entry of the buffer descriptor to the memory loation
        // of the provided RxBufData.
        unsafe { desc.as_mut()[0] = data_addr };

        Self { desc, data }
    }

    /// Returns the value of the `owned` flag of the buffer descriptor,
    /// `true` if the buffer pointed to by the descriptor is owned by the software,
    /// `false` if the buffer is currently in use by the GMAC.
    pub fn owned(&self) -> bool {
        // `owned` flag is bit 0 of word 0 of the descriptor
        let desc = unsafe { self.desc.as_ref() };
        (desc[0] & 0x00000001) != 0
    }

    /// Clears the `owned` flag of the buffer descriptor to return ownership to the GMAC.
    /// Calling this on a descriptor that is currently owned the GMAC has no effect.
    pub fn clear_owned(&mut self) {
        // `owned` flag is bit 0 of word 0 of the descriptor
        let desc = unsafe { self.desc.as_mut() };
        desc[0] &= 0xFFFFFFFE;
    }

    /// Sets the the `wrap` flag of the buffer descriptor to true.
    pub fn set_wrap(&mut self) {
        // `wrap` flag is bit 1 of word 0 of the descriptor
        let desc = unsafe { self.desc.as_mut() };
        desc[0] |= 0x00000002;
    }

    /// Returns the length of data stored in the buffer pointed to by the buffer discriptor.
    /// TODO: This function changes bases on if jumbo frames and fcs are set or not, see datasheet.
    pub fn len(&self) -> usize {
        // If FCS is disabled and jumbo frames are off:
        // `length` of frame excluding FCS is bits 12:0 of word 1 of the buffer descriptor
        let desc = unsafe { self.desc.as_ref() };
        (desc[1] & 0x00000FFF) as usize
    }
}

/// Default zero initialized Tx buffer descriptor.
pub const TX_BUF_DESC_DEFAULT: TxBufDesc = [0; 2];

/// Default zero initialized Tx buffer.
pub const TX_BUF_DATA_DEFAULT: TxBufData = [0; MTU];

pub type TxBufDesc = [u32; 2];
pub type TxBufData = [u8; MTU];

///
pub struct TxBuf {
    pub desc: NonNull<TxBufDesc>,
    pub data: NonNull<TxBufData>,
}

impl TxBuf {
    // Creates a default uninitialized buffer.
    // TODO: Return error instead of assert
    pub fn new(desc: &mut TxBufDesc, data: &mut TxBufData) -> Self {
        let mut desc = NonNull::from(desc);
        let data = NonNull::from(data);

        // TxBufDesc must have an address maskable by 0xFFFFFFFC
        let desc_addr = desc.as_ptr().addr() as u32;
        defmt::assert_eq!(
            desc_addr,
            desc_addr & 0xFFFFFFFC,
            "TxBufDesc memory alignment incorrect"
        );

        // Sets the value of the `addr` entry of the buffer descriptor to the memory loation
        // of the provided TxBufData.
        let data_addr = data.as_ptr().addr() as u32;

        // Sets the value of the `addr` entry of the buffer descriptor to the memory loation
        // of the provided TxBufData.
        unsafe { desc.as_mut()[0] = data_addr };

        // Descriptor must be initialized as owned
        unsafe { desc.as_mut()[1] |= 0x80000000 };

        Self { desc, data }
    }

    /// Returns the value of the `owned` flag of the buffer descriptor,
    /// `true` if the buffer pointed to by the descriptor is owned by the software,
    /// `false` if the buffer is currently in use by the GMAC.
    pub fn owned(&self) -> bool {
        // Datasheet is inconsistant and calls this the `used` flag
        // `used` flag is bit 31 of word 1 of the descriptor
        let desc = unsafe { self.desc.as_ref() };
        (desc[1] & 0x80000000) != 0
    }

    /// Clears the `owned` flag of the buffer descriptor to return ownership to the GMAC.
    /// Calling this on a descriptor that is currently owned the GMAC has no effect.
    pub fn clear_owned(&mut self) {
        // Datasheet is inconsistant and calls this the `used` flag
        // `used` flag is bit 31 of word 1 of the descriptor
        let desc = unsafe { self.desc.as_mut() };
        desc[1] &= 0x7FFFFFFF;
    }

    /// Clears all status bits of the buffer descriptor while maintaining 'owned' and 'wrap' flags.
    pub fn clear_status(&mut self) {
        // `used` flag is bit 31 of word 1 of the descriptor
        let desc = unsafe { self.desc.as_mut() };
        desc[1] &= 0xC0000000;
    }

    /// Sets the the `wrap` flag of the buffer descriptor to true.
    pub fn set_wrap(&mut self) {
        // `wrap` flag is bit 30 of word 1 of the descriptor
        let desc = unsafe { self.desc.as_mut() };
        desc[1] |= 0x40000000;
    }

    /// Sets the `last buffer` flag of the buffer descriptor to true.
    pub fn set_last_buf(&mut self) {
        // `last buffer` flag is bit 15 of word 1 of the descriptor
        let desc = unsafe { self.desc.as_mut() };
        desc[1] |= 0x00008000;
    }

    /// Sets the value of the `len` feild of the buffer descriptor.
    pub fn set_len(&mut self, len: usize) {
        // `len` feild is bits 13:0 of word 1 of the descriptor
        let desc = unsafe { self.desc.as_mut() };
        desc[1] &= 0xFFFFC000;
        desc[1] |= (len as u32) & 0x0003FFF;
    }
}

/// Thin wrapper on RxBuf that represents one received frame.
///
/// On drop RxFrame will return ownership of the buffer to the GMAC
pub struct RxFrame<'a> {
    buf: &'a mut RxBuf,
}

impl<'a> RxFrame<'a> {
    pub fn new(buf: &'a mut RxBuf) -> Self {
        Self { buf }
    }
}

impl<'a> Drop for RxFrame<'a> {
    fn drop(&mut self) {
        self.buf.clear_owned()
    }
}

impl<'a> RxToken for RxFrame<'a> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let data: *mut u8 = self.buf.data.as_ptr().cast();
        let len = self.buf.len();
        let result = f(unsafe { core::slice::from_raw_parts_mut(data, len) });

        // Ensure that our read is complete before returning buffer to GMAC
        compiler_fence(Ordering::SeqCst);

        // Return ownership to GMAC
        self.buf.clear_owned();

        result
    }
}

/// Thin wrapper on TxBuf that represents the capability to transmit one frame.
pub struct TxFrame<'a> {
    buf: &'a mut TxBuf,
    tx_index: &'a mut usize,
}

impl<'a> TxFrame<'a> {
    pub fn new(buf: &'a mut TxBuf, tx_index: &'a mut usize) -> Self {
        Self { buf, tx_index }
    }
}

impl<'a> TxToken for TxFrame<'a> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let data: *mut u8 = self.buf.data.as_ptr().cast();
        let result = f(unsafe { core::slice::from_raw_parts_mut(data, len) });

        // Setup buffer descriptor for send
        self.buf.clear_status();
        self.buf.set_last_buf();
        self.buf.set_len(len);
        self.buf.clear_owned();

        // Ensure that descriptor is set before we call write
        compiler_fence(Ordering::SeqCst);

        // Reach out into the void and notify GMAC to send frame
        let gmac = unsafe { &*GMAC::ptr() };
        gmac.ncr().modify(|_, w| w.tstart().set_bit());

        // Increment the Tx buffer index to point to the next free buffer
        // We are trying to mimic the behavior of the hardware, but there is
        // no guarantee that we will be successful
        *self.tx_index += 1;
        *self.tx_index %= TX_BUF_COUNT;

        result
    }
}
