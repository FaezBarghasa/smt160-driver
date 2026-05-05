#![no_std]

//! # SMT160 Temperature Sensor Driver
//!
//! A high-integrity, `no_std`, async-friendly driver for the SMT160 temperature sensor.
//!
//! ## High-Accuracy Edition
//! This driver is designed to achieve **0.05°C precision** by utilizing high-resolution
//! hardware timers (up to 72MHz) and 64-bit fixed-point arithmetic (`I32F32`).
//!
//! ### The SMT160 Transfer Function
//! The sensor encodes temperature in the duty cycle (DC) of a PWM signal:
//! `DC = 0.320 + 0.00470 * T`
//!
//! Rearranged for temperature calculation:
//! `T = (DC - 0.320) / 0.00470`
//!
//! This driver uses the multiplicative inverse of `0.00470` (approx `212.766`) to 
//! avoid expensive division on platforms without a floating-point unit (FPU).
//!
//! ### Features
//! - **Fixed-Point Engine**: No floating point usage, making it ideal for Cortex-M0/M3/M4.
//! - **Clock-Aware Decoder**: Supports raw timer ticks for sub-microsecond resolution.
//! - **Async Native**: Uses native Rust 2024 async-in-trait support.
//! - **Industrial Failsafes**: Includes outlier rejection, frequency watchdogs, and 16-sample filtering.

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
