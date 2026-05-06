//! Core Data Types for the SMT160 Sensor.

use bitflags::bitflags;
use fixed::types::I16F16;

bitflags! {
    /// Diagnostic operational status of the SMT160 sensor.
    /// 
    /// This bitfield allows for multiple simultaneous warnings or errors 
    /// to be reported in a single telemetry frame.
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
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "Smt160Status({:b})", self.bits());
    }
}

/// A high-precision temperature reading with associated diagnostic metadata.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Reading {
    /// The temperature value in degrees Celsius (°C), represented as a 16.16 fixed-point number.
    pub temperature_celsius: I16F16,
    /// The operational status of the sensor at the exact time of the reading.
    pub status: Smt160Status,
}

/// Health metrics and diagnostic telemetry for the SMT160 sensor subsystem.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Smt160Health {
    /// The Root Mean Square (RMS) jitter of the captured PWM signal in timer ticks.
    pub jitter_rms_ticks: u32,
    /// The measured frequency drift of the sensor signal in Hertz (Hz).
    pub frequency_drift_hz: i32,
    /// The total number of valid samples processed since driver initialization.
    pub total_samples_count: u64,
    /// The total number of processing or hardware errors encountered.
    pub error_total_count: u32,
}
