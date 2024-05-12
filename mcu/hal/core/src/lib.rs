#![no_std]
#![feature(strict_provenance)]
#![feature(type_alias_impl_trait)]

pub use samv71_pac as pac;

pub mod clock;
pub mod dma;
pub mod i2c;
pub mod pio;
pub mod rtt;
pub mod spi;
pub mod uart;
