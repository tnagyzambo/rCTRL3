#![no_std]
#![feature(type_alias_impl_trait)]

pub mod bmi088;
mod spi_bus;

pub use spi_bus::SpiBus;
