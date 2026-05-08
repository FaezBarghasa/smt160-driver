//! Core Data Types for the SMT160 Sensor.
//!
//! This module defines the essential structures and bitfields used for 
//! representing sensor readings, operational status, and health metrics.

use bitflags::bitflags;
use fixed::types::I16F16;

bitflags! {
    /// Diagnostic operational status of the SMT160 sensor.
    /// 
    /// This bitfield allows for multiple simultaneous warnings or errors 
    /// to be reported in a single telemetry frame.
    ///
    /// # Usage Example
    /// ```
    /// use smt160_driver::Smt160Status;
    /// let status = Smt160Status::OK;
    /// if status.contains(Smt160Status::FREQUENCY_ERROR) {
    ///     // Handle error
    /// }
    /// ```
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct Smt160Status: u8 {
        /// Sensor is operating within all nominal parameters.
        const OK = 0b0000_0000;
        /// No edges detected for a significant period (Signal Loss).
        const SIGNAL_LOSS = 0b0000_0001;
        /// PWM frequency is outside the 1kHz-4kHz industrial range.
        const FREQUENCY_ERROR = 0b0000_0010;
        /// Duty cycle is outside the physical 0.320-0.980 bounds.
        const BOUNDARY_VIOLATION = 0b0000_0100;
        /// High jitter detected in the incoming pulse train.
        const JITTER_ALERT = 0b0000_1000;
        /// Temperature reading is out of the specified operating range (-45°C to 130°C).
        const OUT_OF_BOUNDS = 0b0001_0000;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Smt160Status {
    /// Formats the status for `defmt` logging.
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "Smt160Status({:b})", self.bits());
    }
}

/// A high-precision temperature reading with associated diagnostic metadata.
///
/// # Usage Example
/// ```
/// use smt160_driver::Reading;
/// use fixed::types::I16F16;
/// use smt160_driver::Smt160Status;
///
/// let reading = Reading {
///     temperature_celsius: I16F16::from_num(25.5),
///     status: Smt160Status::OK,
/// };
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Reading {
    /// The temperature value in degrees Celsius (°C), represented as a 16.16 fixed-point number.
    pub temperature_celsius: I16F16,
    /// The operational status of the sensor at the exact time of the reading.
    pub status: Smt160Status,
}

/// Advanced diagnostic health metrics for industrial deployments.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IndustrialHealth {
    /// Total valid samples processed since initialization.
    pub total_samples: u64,
    /// Number of signal loss events detected (self-healing triggers).
    pub signal_loss_count: u32,
    /// Number of DMA transfer errors or buffer overruns.
    pub hardware_fault_count: u32,
    /// Current signal jitter in timer ticks (Industrial limit: <50 ticks).
    pub jitter_ticks: u32,
}

/// Real-time performance statistics for system integration validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ProcessingStats {
    /// CPU cycles spent in the last processing batch.
    pub cycles_last_batch: u32,
    /// Maximum CPU cycles recorded in any batch.
    pub cycles_max_observed: u32,
    /// Percentage of CPU utilization (multiplied by 100 for fixed-point).
    pub cpu_load_scaled: u32,
}

