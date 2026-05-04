use fixed::macros::fixed;
use fixed::types::I16F16;

// Formula Constants: T = (DC - 0.320) / 0.00470
pub const DC_OFFSET: I16F16 = fixed!(0.320: I16F16);
pub const DC_STEP: I16F16 = fixed!(0.00470: I16F16);

// Safety Thresholds
pub const MIN_FREQ: u32 = 500;
pub const MAX_FREQ: u32 = 5000;

pub const MIN_DC: I16F16 = fixed!(0.3: I16F16);
pub const MAX_DC: I16F16 = fixed!(0.98: I16F16);

// Industrial Temperature Bounds
pub const MIN_TEMP: I16F16 = fixed!(-45.0: I16F16);
pub const MAX_TEMP: I16F16 = fixed!(130.0: I16F16);

// Jitter Validation (1.5 degrees C)
pub const MAX_JITTER: I16F16 = fixed!(1.5: I16F16);