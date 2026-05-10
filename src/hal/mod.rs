//! Hardware Abstraction Layer for the SMT160 driver.
//!
//! This module encapsulates all platform-specific register manipulations 
//! behind safe traits, ensuring the main driver logic remains clean and testable.

use crate::error::Smt160Error;

pub mod stm32f1_dma;
#[cfg(feature = "rp2040")]
pub mod rp2040_pio;
#[cfg(feature = "stm32g4xx")]
pub mod stm32g4_hrtim;

/// A single captured PWM cycle from the sensor.
///
/// This serves as the data contract between hardware-specific capture 
/// logic and the generic decoding algorithms.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(C)]
pub struct CapturedEdge {
    /// Total duration of the PWM cycle in timer ticks (64-bit for overflow protection).
    pub period_ticks: u64,
    /// Duration of the high phase in timer ticks (64-bit for overflow protection).
    pub high_ticks: u64,
}

/// The core abstraction for SMT160 hardware.
///
/// Any platform wishing to support the SMT160 must implement this trait 
/// using its specific timer and DMA/Interrupt mechanisms.
pub trait Smt160Hal {
    /// Initializes the hardware (clocks, pins, DMA, timers).
    ///
    /// `freq` is the timer clock frequency in Hz, used for internal calculations.
    fn setup(&mut self, freq: u32) -> Result<(), Smt160Error>;

    /// Returns true if the hardware has captured a new, unread PWM cycle.
    fn is_new_data_available(&self) -> bool;

    /// Reads the latest captured edge data from the hardware.
    ///
    /// This should be a non-blocking operation. It is recommended to use 
    /// `#[inline(always)]` on the implementation to minimize overhead.
    fn read_raw(&self) -> CapturedEdge;

    /// Asynchronously waits for a new PWM cycle to be captured.
    ///
    /// This allows the driver to yield control while waiting for hardware 
    /// DMA or interrupt events.
    fn wait_for_new_data(&mut self) -> impl core::future::Future<Output = Result<(), Smt160Error>>;

    /// Notifies the HAL that an interrupt has occurred.
    ///
    /// This should be called from the relevant interrupt handler (e.g., DMA TC/HT)
    /// to wake any pending `wait_for_new_data` futures.
    fn notify(&self);
}
