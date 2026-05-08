//! High-Precision Mathematical Operations for SMT160 Signal Processing.
//!
//! This module provides deterministic, fixed-point mathematical utilities optimized 
//! for industrial-grade PWM decoding without the need for a hardware FPU.

use fixed::types::{I16F16, I32F32};
use crate::Smt160Error;

/// Calculates the frequency of a signal based on its period and clock speed.
/// 
/// # Summary
/// This function derives the frequency in Hertz (Hz) from the captured period ticks 
/// and the known timer frequency.
/// 
/// # Errors
/// Returns `Smt160Error::InvalidSignal` if `period_ticks` is zero.
/// 
/// # Usage Example
/// ```
/// use smt160_driver::math::calculate_frequency_hz;
/// let freq = calculate_frequency_hz(1000, 72).unwrap();
/// assert_eq!(freq, 72_000);
/// ```
pub fn calculate_frequency_hz(period_ticks: u64, timer_clock_megahertz: u32) -> Result<u64, Smt160Error> {
    (timer_clock_megahertz as u64 * 1_000_000)
        .checked_div(period_ticks)
        .ok_or(Smt160Error::InvalidSignal)
}

/// Calculates the duty cycle from active and period ticks.
/// 
/// # Summary
/// Returns a high-precision `I32F32` fixed-point representation of the duty cycle.
/// 
/// # Errors
/// Returns `Smt160Error::InvalidSignal` if `period_ticks` is zero.
/// 
/// # Usage Example
/// ```
/// use smt160_driver::math::calculate_duty_cycle;
/// use fixed::types::I32F32;
/// let dc = calculate_duty_cycle(50, 100).unwrap();
/// assert_eq!(dc, I32F32::from_num(0.5));
/// ```
pub fn calculate_duty_cycle(active_ticks: u64, period_ticks: u64) -> Result<I32F32, Smt160Error> {
    if period_ticks == 0 {
        return Err(Smt160Error::InvalidSignal);
    }
    I32F32::from_num(active_ticks)
        .checked_div(I32F32::from_num(period_ticks))
        .ok_or(Smt160Error::InvalidSignal)
}

/// Calculates the final temperature from a duty cycle using sensor constants.
/// 
/// # Summary
/// Implements the core SMT160 transfer function: `T = (DutyCycle - Offset) * InverseStep`.
/// 
/// # Usage Example
/// ```
/// use smt160_driver::math::calculate_temperature_celsius;
/// use fixed::types::{I16F16, I32F32};
/// let temp = calculate_temperature_celsius(
///     I32F32::from_num(0.4375), 
///     I32F32::from_num(0.32), 
///     I32F32::from_num(212.76)
/// );
/// ```
pub fn calculate_temperature_celsius(
    current_duty_cycle: I32F32,
    duty_cycle_offset: I32F32,
    inverse_step_constant: I32F32,
) -> I16F16 {
    I16F16::from_num((current_duty_cycle - duty_cycle_offset) * inverse_step_constant)
}

/// Performs piecewise linear interpolation between two points.
/// 
/// # Summary
/// This function calculates the value `y` for a given `x` based on a line segment 
/// defined by two points `(x1, y1)` and `(x2, y2)`.
/// 
/// # Errors
/// Returns `y1_value` if `x1_point == x2_point` to avoid division by zero.
/// 
/// # Panics
/// This function does not panic.
/// 
/// # Usage Example
/// ```
/// use fixed::types::{I16F16, I32F32};
/// use smt160_driver::math::interpolate_linear;
/// let x = I32F32::from_num(0.5);
/// let p1 = (I32F32::from_num(0.0), I16F16::from_num(10));
/// let p2 = (I32F32::from_num(1.0), I16F16::from_num(20));
/// let result = interpolate_linear(x, p1.0, p1.1, p2.0, p2.1);
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

/// Applies an Exponentially Weighted Moving Average (EWMA) filter using bit-shifts.
/// 
/// # Summary
/// Updates the current average using a shift factor `alpha_shift`.
/// `NewAverage = OldAverage + ((NewValue - OldAverage) >> alpha_shift)`
/// 
/// # Panics
/// This function does not panic.
/// 
/// # Usage Example
/// ```
/// use fixed::types::I16F16;
/// use smt160_driver::math::apply_shift_ema_filter;
/// let current_average = I16F16::from_num(25.0);
/// let new_reading = I16F16::from_num(26.0);
/// let updated_average = apply_shift_ema_filter(current_average, new_reading, 3);
/// ```
pub fn apply_shift_ema_filter(
    current_average: I16F16,
    new_reading: I16F16,
    alpha_shift: u32,
) -> I16F16 {
    current_average + ((new_reading - current_average) >> alpha_shift)
}

/// Applies a 2nd-order quadratic linearity correction.
/// 
/// # Summary
/// Calculates `T_corr = T + 0.0015 * (T - 25)^2` for readings outside the 
/// 10°C to 50°C range to improve accuracy across the full spectrum.
/// 
/// # Panics
/// This function does not panic.
/// 
/// # Usage Example
/// ```
/// use fixed::types::I16F16;
/// use smt160_driver::math::apply_linearity_correction;
/// let temp = I16F16::from_num(5.0);
/// let corrected = apply_linearity_correction(temp);
/// ```
pub fn apply_linearity_correction(
    temperature_celsius: I16F16,
) -> I16F16 {
    let lower_bound = I16F16::from_num(10);
    let upper_bound = I16F16::from_num(50);
    
    if temperature_celsius < lower_bound || temperature_celsius > upper_bound {
        let diff = temperature_celsius - I16F16::from_num(25);
        let diff_sq = diff * diff;
        let correction = I16F16::from_num(0.0015) * diff_sq;
        temperature_celsius + correction
    } else {
        temperature_celsius
    }
}

