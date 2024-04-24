pub type Hz = usize;

pub trait Clock {
    /// The nominal frequency of the clock source
    const FREQ: Hz;
}

mod mainck;
pub mod mck;
mod slck;

pub use mainck::{Mainck, MainckSource};
pub use mck::Mck;
pub use slck::{Slck, SlckSource};
