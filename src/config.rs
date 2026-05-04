//! Configuration constants and physical constraints for the SMT160 sensor.

use fixed::types::I16F16;

/// Duty Cycle offset constant from the SMT160 transfer function.
/// Formula: DC = 0.320 + 0.00470 * T
pub const DC_OFFSET: I16F16 = I16F16::from_bits(20972); // 0.320

/// Duty Cycle step (sensitivity) constant from the SMT160 transfer function.
pub const DC_STEP: I16F16 = I16F16::from_bits(308); // 0.00470

/// Minimum allowable output frequency (Hz) for a valid sensor signal.
pub const MIN_FREQ: u32 = 500;
/// Maximum allowable output frequency (Hz) for a valid sensor signal.
pub const MAX_FREQ: u32 = 5000;

/// Minimum physical duty cycle (below this is considered an error).
pub const MIN_DC: I16F16 = I16F16::from_bits(19661); // 0.3
/// Maximum physical duty cycle (above this is considered an error).
pub const MAX_DC: I16F16 = I16F16::from_bits(64225); // 0.98

/// Industrial lower bound for temperature (°C).
pub const MIN_TEMP: I16F16 = I16F16::from_bits(-2949120); // -45.0
/// Industrial upper bound for temperature (°C).
pub const MAX_TEMP: I16F16 = I16F16::from_bits(8519680); // 130.0

/// Maximum allowed deviation (°C) from the rolling average for jitter filtering.
pub const MAX_JITTER: I16F16 = I16F16::from_bits(98304); // 1.5