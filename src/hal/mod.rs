//! Hardware Abstraction Layer for the SMT160 driver.
//!
//! This module encapsulates all platform-specific register manipulations 
//! behind safe traits, ensuring the main driver logic remains clean and testable.

pub mod stm32f1_dma;

pub use stm32f1_dma::{
    validate_clocks,
    Smt160TimerInstance,
    Smt160DmaChannel,
};
