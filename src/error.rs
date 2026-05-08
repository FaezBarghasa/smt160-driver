//! Unified Error Handling for the SMT160 Driver.
//!
//! This module defines the `Smt160Error` enum, which aggregates all possible 
//! failure modes from hardware capture to mathematical processing.

use core::fmt;

/// All potential failures of the SMT160 processing pipeline.
/// 
/// This enum provides a unified error type for hardware capture, 
/// signal decoding, and configuration management.
///
/// # Usage Example
/// ```
/// use smt160_driver::Smt160Error;
/// let err = Smt160Error::Timeout;
/// println!("Error: {}", err);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Smt160Error {
    /// The sensor signal timed out (no edges detected within the required window).
    Timeout,
    /// The calculated duty cycle is outside the physical bounds of the sensor (0.320-0.980).
    InvalidDutyCycle,
    /// The PWM frequency is outside the specified 1kHz-4kHz operating range.
    FrequencyOutOfRange,
    /// The calculated temperature is outside the sensor's industrial operating range (-45°C to 130°C).
    ThermalOverload,
    /// The edge sequence (Rise -> Fall -> Rise) was violated or inconsistent.
    SequenceViolation,
    /// The signal jitter exceeds the safety threshold for high-accuracy sensing.
    HighJitter,
    /// An error occurred during I2C telemetry communication.
    I2cError,
    /// The provided configuration is invalid or inconsistent.
    InvalidConfiguration,
    /// The incoming raw signal is mathematically invalid (e.g., division by zero).
    InvalidSignal,
}

impl fmt::Display for Smt160Error {
    /// Formats the error for human-readable display.
    /// 
    /// # Errors
    /// This function only fails if the underlying formatter fails.
    ///
    /// # Panics
    /// This function does not panic.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Timeout => write!(f, "Sensor Signal Timeout"),
            Self::InvalidDutyCycle => write!(f, "Invalid Duty Cycle Detected"),
            Self::FrequencyOutOfRange => write!(f, "PWM Frequency Out of Operational Range"),
            Self::ThermalOverload => write!(f, "Thermal Overload: Temperature Out of Bounds"),
            Self::SequenceViolation => write!(f, "Signal Sequence Violation"),
            Self::HighJitter => write!(f, "Signal Jitter Exceeds Safety Threshold"),
            Self::I2cError => write!(f, "I2C Telemetry Communication Error"),
            Self::InvalidConfiguration => write!(f, "Invalid Driver Configuration"),
            Self::InvalidSignal => write!(f, "Mathematically Invalid Signal Capture"),
        }
    }
}



