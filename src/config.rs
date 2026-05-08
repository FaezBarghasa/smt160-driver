//! Configuration Constants and Physical Constraints for SMT160.

use fixed::types::{I32F32, I16F16};

/// Duty Cycle offset constant from the standard SMT160 transfer function.
/// Formula: DutyCycle = 0.320 + 0.00470 * Temperature_Celsius
pub const DUTY_CYCLE_OFFSET: I32F32 = I32F32::from_bits(1374389535); // 0.320

/// Pre-calculated inverse of the step constant (1 / 0.00470).
/// Used for high-performance multiplication on platforms without an FPU.
/// 1 / 0.00470 = 212.765957...
pub const INVERSE_STEP_CONSTANT: I32F32 = I32F32::from_bits(913822833441); // 212.765957...

/// Minimum allowable signal frequency in Hertz (Hz) for safety validation.
pub const MINIMUM_FREQUENCY_HZ: u32 = 500;

/// Maximum allowable signal frequency in Hertz (Hz) for safety validation.
pub const MAXIMUM_FREQUENCY_HZ: u32 = 5000;

/// Lower industrial bound for temperature measurement in degrees Celsius (°C).
pub const MINIMUM_TEMPERATURE_CELSIUS: I16F16 = I16F16::from_bits(-2949120); // -45.0

/// Upper industrial bound for temperature measurement in degrees Celsius (°C).
pub const MAXIMUM_TEMPERATURE_CELSIUS: I16F16 = I16F16::from_bits(8519680); // 130.0

/// Trait for providing SMT160 configuration and calibration parameters.
pub trait Smt160Config {
    /// Returns the (Duty Cycle Offset, Inverse Step Constant) for temperature calculation.
    fn get_offsets(&self) -> (I32F32, I32F32);
}

/// A hardcoded configuration using standard manufacturer-specified SMT160 constants.
pub struct StaticConfiguration;

impl Smt160Config for StaticConfiguration {
    /// Returns the standard SMT160 transfer function constants.
    fn get_offsets(&self) -> (I32F32, I32F32) {
        (DUTY_CYCLE_OFFSET, INVERSE_STEP_CONSTANT)
    }
}

/// Macro for defining a hardcoded SMT160 configuration.
/// 
/// # Usage Example
/// ```
/// use smt160_driver::smt160_config;
/// smt160_config!(MyConfig, 0.320, 212.76);
/// ```
#[macro_export]
macro_rules! smt160_config {
    ($name:ident, $offset:expr, $step:expr) => {
        pub struct $name;
        impl $crate::config::Smt160Config for $name {
            fn get_offsets(&self) -> ($crate::fixed::types::I32F32, $crate::fixed::types::I32F32) {
                ($crate::fixed::types::I32F32::from_num($offset), $crate::fixed::types::I32F32::from_num($step))
            }
        }
    };
}