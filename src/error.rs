//! Error and Status enumerations for the SMT160 driver.
//!
//! This module defines the core error types used throughout the driver, 
//! designed for industrial safety and efficient telemetry.

use core::fmt;

/// All possible error conditions for the SMT160 driver.
///
/// This enum is designed to be compatible with `defmt` for efficient, 
/// deferred logging in industrial environments where jitter must be minimized.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Smt160Error {
    /// The system clock or timer clock is below the 8MHz minimum required 
    /// to achieve the 0.05°C precision target. High-speed signals require 
    /// high-resolution timers.
    ClockTooSlow,
    
    /// The provided DMA buffer or slice is of an invalid size or alignment.
    /// This prevents memory corruption during high-speed burst transfers.
    InvalidBuffer,
    
    /// The sensor failed to pulse within the expected 5ms window (sensor max period is ~1ms).
    /// This usually indicates a disconnected sensor, a broken wire, or a hardware ESD freeze.
    SensorTimeout,
    
    /// The calculated temperature or duty cycle is physically impossible 
    /// (e.g., outside the -45°C to +130°C range). This can also occur 
    /// if the signal is extremely noisy (jitter exceeding correction limits).
    OutOfBounds,
}

impl fmt::Display for Smt160Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ClockTooSlow => write!(f, "Clock frequency insufficient for 0.05°C precision"),
            Self::InvalidBuffer => write!(f, "Invalid DMA buffer configuration"),
            Self::SensorTimeout => write!(f, "Sensor pulse timeout (disconnected or ESD freeze)"),
            Self::OutOfBounds => write!(f, "Measurement out of bounds or signal corrupted"),
        }
    }
}
