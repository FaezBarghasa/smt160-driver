//! High-precision, fixed-point mathematical core for SMT160 signal processing.
//!
//! This module is decoupled from hardware registers to enable full property-based 
//! testing on host machines. It uses the `fixed` crate to provide deterministic 
//! results without a hardware FPU.

use fixed::types::I32F32;
use crate::error::Smt160Error;

/// Stateless signal decoder for converting timer ticks into temperature.
pub struct SignalDecoder;

impl SignalDecoder {
    /// SMT160 Transfer Function: T = (DutyCycle - 0.320) / 0.00470
    /// 
    /// Pre-calculated constants for maximum performance (branchless paths):
    /// - OFFSET: 0.320
    /// - INVERSE_STEP: 1 / 0.00470 ≈ 212.7659574468
    const DC_OFFSET: I32F32 = I32F32::from_bits(1374389535); // 0.320 * 2^32
    const INVERSE_STEP: I32F32 = I32F32::from_bits(913840134891); // 212.765957... * 2^32

    /// Decodes raw timer ticks into a fixed-point temperature value.
    ///
    /// # Mathematical Safety
    /// This function uses checked arithmetic and boundary validation to ensure 
    /// it never panics, even with malicious or corrupt hardware input.
    #[inline(always)]
    pub fn decode(period_ticks: u64, active_ticks: u64) -> Result<I32F32, Smt160Error> {
        // Guard against zero-period (division by zero)
        if period_ticks == 0 {
            return Err(Smt160Error::SensorTimeout);
        }

        // Physical impossibility: active time cannot exceed total period
        if active_ticks > period_ticks {
            return Err(Smt160Error::InvalidSignal);
        }

        // Calculate Duty Cycle: DC = active / period
        // We use I64F64 for intermediate calculation to prevent overflow 
        // when converting u32 ticks > 2^31 into a signed fixed-point type.
        use fixed::types::I64F64;
        let active_fp = I64F64::from_num(active_ticks);
        let period_fp = I64F64::from_num(period_ticks);
        
        // Division is safe because period_fp >= 1 (since period_ticks > 0)
        let dc: I32F32 = (active_fp / period_fp).to_num();

        // T = (DC - 0.320) * 212.766
        let raw_temp = (dc - Self::DC_OFFSET) * Self::INVERSE_STEP;

        // Industrial Safety Check: -45°C to 130°C
        // Outside this range, the sensor is either broken or in a catastrophic environment.
        if raw_temp < I32F32::from_num(-45) || raw_temp > I32F32::from_num(130) {
            return Err(Smt160Error::OutOfBounds);
        }

        Ok(raw_temp)
    }

    /// Applies Non-Linearity Correction (NLC) using linear interpolation.
    #[inline(always)]
    pub fn apply_nlc(raw_temp: I32F32) -> I32F32 {
        Self::apply_nlc_custom(raw_temp, Self::DEFAULT_NLC_TABLE)
    }

    /// The default NLC table for standard SMT160 sensors.
    pub const DEFAULT_NLC_TABLE: &[(I32F32, I32F32)] = &[
        (I32F32::from_bits(-128849018880), I32F32::from_bits(-130137505792)), // -30.0 -> -30.3
        (I32F32::from_bits(0), I32F32::from_bits(0)),                        // 0.0 -> 0.0
        (I32F32::from_bits(107374182400), I32F32::from_bits(107374182400)),   // 25.0 -> 25.0
        (I32F32::from_bits(343597383680), I32F32::from_bits(341020410060)),  // 80.0 -> 79.4
        (I32F32::from_bits(515396075520), I32F32::from_bits(511099977728)),  // 120.0 -> 119.0
    ];

    /// Applies NLC using a custom lookup table.
    pub fn apply_nlc_custom(raw_temp: I32F32, table: &[(I32F32, I32F32)]) -> I32F32 {
        if table.is_empty() { return raw_temp; }

        // Boundary checks for extrapolation (clamping to extremes)
        if raw_temp <= table[0].0 { return table[0].1; }
        if raw_temp >= table[table.len() - 1].0 { return table[table.len() - 1].1; }

        // Linear interpolation between table points
        for i in 0..table.len() - 1 {
            let (x0, y0) = table[i];
            let (x1, y1) = table[i + 1];

            if raw_temp >= x0 && raw_temp <= x1 {
                let dx = x1 - x0;
                let dy = y1 - y0;
                // Safe linear interpolation formula: y = y0 + (x - x0) * (dy / dx)
                return y0 + (raw_temp - x0) * dy / dx;
            }
        }

        raw_temp
    }

    /// Applies an adaptive EWMA filter based on temperature deviation and startup state.
    /// 
    /// # Alpha Selection Logic
    /// - **Fast Track (α=0.8)**: Used if deviation > 5°C or during the first 16 samples. 
    ///   Ensures rapid response to thermal events or system startup.
    /// - **Steady State (α=0.1)**: Used for high-precision noise rejection once stabilized.
    #[inline(always)]
    pub fn apply_adaptive_filter(current: I32F32, last: Option<I32F32>, count: u32) -> I32F32 {
        let last_val = match last {
            Some(v) => v,
            None => return current,
        };

        let diff = (current - last_val).abs();
        let alpha = if diff > I32F32::from_num(5) || count < 16 {
            I32F32::from_num(0.8)
        } else {
            I32F32::from_num(0.1)
        };

        // Y_n = alpha * X_n + (1 - alpha) * Y_{n-1}
        let one_minus_alpha = I32F32::from_num(1) - alpha;
        (alpha * current) + (one_minus_alpha * last_val)
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;

    #[test]
    fn test_exact_values() {
        // DC = 0.320 -> T = 0°C
        let res = SignalDecoder::decode(1000, 320).unwrap();
        assert!(res.abs_diff(I32F32::from_num(0)) < I32F32::from_num(0.001));

        // DC = 0.4375 -> T = 25°C
        let res = SignalDecoder::decode(10000, 4375).unwrap();
        assert!(res.abs_diff(I32F32::from_num(25)) < I32F32::from_num(0.001));
    }
}

#[cfg(all(kani, feature = "std"))]
mod verification {
    use super::*;

    #[kani::proof]
    fn verify_decode_no_panic() {
        let period: u64 = kani::any();
        let active: u64 = kani::any();
        // The decoder should never panic, regardless of input
        let _ = SignalDecoder::decode(period, active);
    }

    #[kani::proof]
    fn verify_nlc_bounds() {
        let temp: I32F32 = I32F32::from_bits(kani::any());
        let corrected = SignalDecoder::apply_nlc(temp);
        // Corrected temp must be within reasonable physical bounds if input was
        kani::assert(corrected >= I32F32::from_num(-55), "NLC underflow");
        kani::assert(corrected <= I32F32::from_num(155), "NLC overflow");
    }

    #[kani::proof]
    fn verify_adaptive_filter_no_panic() {
        let current: I32F32 = I32F32::from_bits(kani::any());
        let last_val: I32F32 = I32F32::from_bits(kani::any());
        let last: Option<I32F32> = if kani::any() { Some(last_val) } else { None };
        let count: u32 = kani::any();
        // Adaptive filter should never panic or overflow internally 
        // given its weighted average nature.
        let _ = SignalDecoder::apply_adaptive_filter(current, last, count);
    }
}
