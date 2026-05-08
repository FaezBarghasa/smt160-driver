//! Hardware Abstraction Layer for the SMT160 driver.
//!
//! This module contains platform-specific implementations and traits 
//! to decouple the core driver from the underlying microcontroller peripherals.

#[cfg(feature = "stm32f1xx")]
pub mod stm32f1_dma;

#[cfg(feature = "stm32f1xx")]
pub use stm32f1_dma::*;
