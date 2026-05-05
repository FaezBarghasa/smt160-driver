#![no_std]

//! # SMT160 Temperature Sensor Driver
//!
//! A high-integrity, `no_std`, async-friendly driver for the SMT160 temperature sensor.
//! This driver uses fixed-point math (`I16F16`) for deterministic performance on 
//! microcontrollers and includes advanced jitter filtering and safety checks.
//!
//! ## Features
//! - **Fixed-Point Engine**: No floating point usage, making it ideal for Cortex-M0/M3/M4.
//! - **Async Native**: Supports `embedded-hal-async` for non-blocking operation.
//! - **Passive Decoder**: A state machine that can be driven by interrupts or polling.
//! - **Failsafe Mechanisms**: Includes jitter filtering, frequency watchdogs, and stability counters.

pub mod config;
pub mod decoder;
pub mod driver_async;
pub mod driver_blocking;
pub mod i2c_telemetry;

use fixed::types::I16F16;

/// All potential failures of the SMT160 processing pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Smt160Error {
    /// No edge was detected within the expected timeframe.
    Timeout,
    /// Calculated duty cycle is outside the physical bounds of the sensor (0.3 - 0.98).
    InvalidDutyCycle(I16F16),
    /// Sensor frequency is outside the specified 500Hz - 5000Hz range.
    FrequencyOutOfRange(u32),
    /// Detected temperature is outside the industrial range (-45°C to 130°C).
    ThermalOverload(I16F16),
    /// Hardware sequence violation (e.g., two rising edges without a falling edge) 
    /// or a sudden frequency shift >10%.
    SequenceViolation,
    /// High jitter detected: value deviates >1.5°C from the rolling average.
    HighJitter,
    /// Error during I2C communication.
    I2cError,
}

impl core::fmt::Display for Smt160Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Timeout => write!(f, "Timeout waiting for sensor edge"),
            Self::InvalidDutyCycle(dc) => write!(f, "Duty cycle out of bounds: {}", dc),
            Self::FrequencyOutOfRange(freq) => write!(f, "Frequency out of range: {} Hz", freq),
            Self::ThermalOverload(temp) => write!(f, "Thermal overload detected: {} C", temp),
            Self::SequenceViolation => write!(f, "Hardware sequence violation or 10% frequency shift"),
            Self::HighJitter => write!(f, "High jitter detected (>1.5C deviation)"),
            Self::I2cError => write!(f, "I2C communication error"),
        }
    }
}
