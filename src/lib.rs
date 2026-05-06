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
#[cfg(feature = "stm32f1")]
pub mod stm32f1;
pub mod calibration;
pub mod telemetry;

use fixed::types::I16F16;

use bitflags::bitflags;

bitflags! {
    /// Diagnostic status of the SMT160 sensor.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct Smt160Status: u8 {
        /// Sensor is operating normally.
        const OK = 0b0000_0000;
        /// No edges detected for a significant period (Signal Loss).
        const SIGNAL_LOSS = 0b0000_0001;
        /// PWM frequency is outside the 1kHz-4kHz range.
        const FREQUENCY_ERROR = 0b0000_0010;
        /// Duty cycle is outside the physical 0.320-0.980 bounds.
        const BOUNDARY_VIOLATION = 0b0000_0100;
        /// High jitter detected in the incoming signal.
        const JITTER_ALERT = 0b0000_1000;
        /// Temperature reading is out of specified operating range.
        const OUT_OF_BOUNDS = 0b0001_0000;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Smt160Status {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "Smt160Status({:b})", self.bits());
    }
}

// Backward compatibility or simpler naming if needed
pub type SensorStatus = Smt160Status;

/// A temperature reading with diagnostic metadata.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Reading {
    /// The temperature value in Celsius (°C).
    pub value: I16F16,
    /// The operational status of the sensor at the time of reading.
    pub status: SensorStatus,
}

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
    InvalidConfiguration,
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
            Self::InvalidConfiguration => write!(f, "Invalid Configuration"),
        }
    }
}
