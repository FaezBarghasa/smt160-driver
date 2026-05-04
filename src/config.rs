//! Configuration constants and physical constraints for the SMT160 sensor.

use fixed::macros::fixed;
use fixed::types::I16F16;

/// Duty Cycle offset constant from the SMT160 transfer function.
/// Formula: DC = 0.320 + 0.00470 * T
pub const DC_OFFSET: I16F16 = fixed!(0.320: I16F16);

/// Duty Cycle step (sensitivity) constant from the SMT160 transfer function.
pub const DC_STEP: I16F16 = fixed!(0.00470: I16F16);

/// Minimum allowable output frequency (Hz) for a valid sensor signal.
pub const MIN_FREQ: u32 = 500;
/// Maximum allowable output frequency (Hz) for a valid sensor signal.
pub const MAX_FREQ: u32 = 5000;

/// Minimum physical duty cycle (below this is considered an error).
pub const MIN_DC: I16F16 = fixed!(0.3: I16F16);
/// Maximum physical duty cycle (above this is considered an error).
pub const MAX_DC: I16F16 = fixed!(0.98: I16F16);

/// Industrial lower bound for temperature (°C).
pub const MIN_TEMP: I16F16 = fixed!(-45.0: I16F16);
/// Industrial upper bound for temperature (°C).
pub const MAX_TEMP: I16F16 = fixed!(130.0: I16F16);

/// Maximum allowed deviation (°C) from the rolling average for jitter filtering.
pub const MAX_JITTER: I16F16 = fixed!(1.5: I16F16);