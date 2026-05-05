//! Configuration constants and physical constraints for the SMT160 sensor.

use fixed::types::{I32F32, I16F16};

/// Duty Cycle offset constant from the SMT160 transfer function.
/// Formula: DC = 0.320 + 0.00470 * T
pub const DC_OFFSET: I32F32 = I32F32::from_num(0.320);

/// Pre-calculated inverse of the step constant (1 / 0.00470)
/// for high-performance multiplication on platforms without FPU.
/// 1 / 0.00470 = 212.7659574468085
pub const INV_STEP: I32F32 = I32F32::from_num(212.7659574468085);

/// Safety thresholds for valid sensor signal (Hz)
pub const MIN_FREQ: u32 = 500;
pub const MAX_FREQ: u32 = 5000;

/// Industrial bounds for temperature (°C).
pub const MIN_TEMP: I16F16 = I16F16::from_bits(-2949120); // -45.0
pub const MAX_TEMP: I16F16 = I16F16::from_bits(8519680); // 130.0