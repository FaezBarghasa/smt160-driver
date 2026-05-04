#![no_std]

pub mod config;
pub mod decoder;
pub mod driver_async;

use fixed::types::I16F16;

/// All potential failures of the SMT160 processing pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Smt160Error {
    Timeout,
    InvalidDutyCycle(I16F16),
    FrequencyOutOfRange(u32),
    ThermalOverload(I16F16),
    SequenceViolation,
    HighJitter,
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
        }
    }
}