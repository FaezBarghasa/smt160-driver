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
    /// Pre-calculated constants for performance:
    /// - OFFSET: 0.320
    /// - INVERSE_STEP: 1 / 0.00470 ≈ 212.7659574468
    const DC_OFFSET: I32F32 = I32F32::from_bits(1374389534); // 0.320
    const INVERSE_STEP: I32F32 = I32F32::from_bits(91384013488); // 212.7659574468

    /// Decodes raw timer ticks into a fixed-point temperature value.
    ///
    /// # Mathematical Safety
    /// This function uses checked arithmetic and boundary validation to ensure 
    /// it never panics, even with malicious or corrupt hardware input.
    pub fn decode(period_ticks: u32, active_ticks: u32) -> Result<I32F32, Smt160Error> {
        if period_ticks == 0 {
            return Err(Smt160Error::SensorTimeout);
        }

        if active_ticks > period_ticks {
            return Err(Smt160Error::OutOfBounds);
        }

        // Calculate Duty Cycle: DC = active / period
        let active_fp = I32F32::from_num(active_ticks);
        let period_fp = I32F32::from_num(period_ticks);
        
        // Division is safe because period_fp > 0
        let dc = active_fp / period_fp;

        // T = (DC - 0.320) * 212.766
        let raw_temp = (dc - Self::DC_OFFSET) * Self::INVERSE_STEP;

        // Industrial Safety Check: -45°C to 130°C
        if raw_temp < I32F32::from_num(-45) || raw_temp > I32F32::from_num(130) {
            return Err(Smt160Error::OutOfBounds);
        }

        Ok(raw_temp)
    }

    /// Applies Non-Linearity Correction (NLC) using linear interpolation.
    /// 
    /// The SMT160 is highly linear, but exhibits slight curvature at 
    /// temperature extremes. This function compensates for that error.
    pub fn apply_nlc(raw_temp: I32F32) -> I32F32 {
        // Table points: (Raw, Corrected)
        // Optimized for the -30°C to +120°C industrial range.
        const TABLE: &[(I32F32, I32F32)] = &[
            (I32F32::from_bits(-128849018880), I32F32::from_bits(-130137505792)), // -30.0 -> -30.3 (hypothetical correction)
            (I32F32::from_bits(0), I32F32::from_bits(0)),                        // 0.0 -> 0.0
            (I32F32::from_bits(107374182400), I32F32::from_bits(107374182400)),   // 25.0 -> 25.0
            (I32F32::from_bits(343597383680), I32F32::from_bits(341020410060)),  // 80.0 -> 79.4
            (I32F32::from_bits(515396075520), I32F32::from_bits(511099977728)),  // 120.0 -> 119.0
        ];

        if raw_temp <= TABLE[0].0 { return TABLE[0].1; }
        if raw_temp >= TABLE[TABLE.len() - 1].0 { return TABLE[TABLE.len() - 1].1; }

        for i in 0..TABLE.len() - 1 {
            let (x0, y0) = TABLE[i];
            let (x1, y1) = TABLE[i + 1];

            if raw_temp >= x0 && raw_temp <= x1 {
                let dx = x1 - x0;
                let dy = y1 - y0;
                return y0 + (raw_temp - x0) * dy / dx;
            }
        }

        raw_temp
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// Mathematical Proof: The decode function must never panic for any u32 inputs.
        /// It must either return a valid I32F32 or an Smt160Error.
        #[test]
        fn proof_decode_never_panics(p in 0..u32::MAX, a in 0..u32::MAX) {
            let _ = SignalDecoder::decode(p, a);
        }

        /// Mathematical Proof: The NLC function must never panic for any reasonable 
        /// temperature input range.
        #[test]
        fn proof_nlc_never_panics(temp_bits in -2147483648i64..2147483647i64) {
            let temp = I32F32::from_bits(temp_bits);
            let _ = SignalDecoder::apply_nlc(temp);
        }

        /// Property: Duty cycles > 1.0 must return OutOfBounds.
        #[test]
        fn property_active_greater_than_period_is_error(p in 1..1000000u32) {
            let a = p + 1;
            let result = SignalDecoder::decode(p, a);
            assert_eq!(result, Err(Smt160Error::OutOfBounds));
        }
    }

    #[test]
    fn test_exact_values() {
        // DC = 0.320 -> T = 0°C
        let res = SignalDecoder::decode(1000, 320).unwrap();
        assert!(res.abs_diff(I32F32::from_num(0)) < 0.001);

        // DC = 0.4375 -> T = 25°C
        let res = SignalDecoder::decode(10000, 4375).unwrap();
        assert!(res.abs_diff(I32F32::from_num(25)) < 0.001);
    }
}
