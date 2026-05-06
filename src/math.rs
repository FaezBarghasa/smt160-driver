//! High-Precision Mathematical Operations for SMT160 Signal Processing.

use fixed::types::{I16F16, I32F32};

/// Performs piecewise linear interpolation between two points.
/// 
/// # Summary
/// This function calculates the value `y` for a given `x` based on a line segment 
/// defined by two points `(x1, y1)` and `(x2, y2)`.
/// 
/// # Errors
/// Returns `I16F16::ZERO` if `x1 == x2` to avoid division by zero.
/// 
/// # Usage Example
/// ```
/// let x = I32F32::from_num(0.5);
/// let p1 = (I32F32::from_num(0.0), I16F16::from_num(10));
/// let p2 = (I32F32::from_num(1.0), I16F16::from_num(20));
/// let result = smt160_driver::math::interpolate_linear(x, p1.0, p1.1, p2.0, p2.1);
/// assert_eq!(result, I16F16::from_num(15));
/// ```
pub fn interpolate_linear(
    x_input: I32F32,
    x1_point: I32F32,
    y1_value: I16F16,
    x2_point: I32F32,
    y2_value: I16F16,
) -> I16F16 {
    let delta_x = x2_point - x1_point;
    if delta_x == 0 {
        return y1_value;
    }
    
    let delta_y = I32F32::from_num(y2_value) - I32F32::from_num(y1_value);
    let interpolated_value = I32F32::from_num(y1_value) + (x_input - x1_point) * delta_y / delta_x;
    
    I16F16::from_num(interpolated_value)
}

/// Applies an Exponentially Weighted Moving Average (EWMA) filter.
/// 
/// # Summary
/// Updates the current average using a smoothing factor `alpha`.
/// `NewAverage = alpha * NewValue + (1 - alpha) * OldAverage`
/// 
/// # Usage Example
/// ```
/// let current_average = I16F16::from_num(25.0);
/// let new_reading = I16F16::from_num(26.0);
/// let alpha = I32F32::from_num(0.1);
/// let updated_average = smt160_driver::math::apply_ewma_filter(current_average, new_reading, alpha);
/// ```
pub fn apply_ewma_filter(
    current_average: I16F16,
    new_reading: I16F16,
    alpha_smoothing: I32F32,
) -> I16F16 {
    let one_minus_alpha = I32F32::ONE - alpha_smoothing;
    let filtered_value = alpha_smoothing * I32F32::from_num(new_reading) 
                         + one_minus_alpha * I32F32::from_num(current_average);
    
    I16F16::from_num(filtered_value)
}
