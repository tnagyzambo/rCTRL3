#![no_std]
#![feature(strict_provenance)]
#![feature(generic_const_exprs)]
pub use samv71_pac as pac;

pub mod clock;
pub mod dma;
pub mod i2c;
pub mod pio;
pub mod rtt;
pub mod uart;
