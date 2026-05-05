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

// use fixed::types::I16F16;

/// All potential failures of the SMT160 processing pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Smt160Error {
    Timeout,
    InvalidDutyCycle,
    FrequencyOutOfRange,
    ThermalOverload,
    SequenceViolation,
    HighJitter,
    I2cError,
}

impl core::fmt::Display for Smt160Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Timeout => write!(f, "Timeout"),
            Self::InvalidDutyCycle => write!(f, "Invalid Duty Cycle"),
            Self::FrequencyOutOfRange => write!(f, "Frequency Out Of Range"),
            Self::ThermalOverload => write!(f, "Thermal Overload"),
            Self::SequenceViolation => write!(f, "Sequence Violation"),
            Self::HighJitter => write!(f, "High Jitter"),
            Self::I2cError => write!(f, "I2C Error"),
        }
    }
}
