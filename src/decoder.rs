//! The SMT160 logic engine.
//!
//! This module implements a passive state machine that calculates temperature from 
//! pulse-width modulated (PWM) signal timestamps.

use crate::config::*;
use crate::Smt160Error;
use fixed::types::I16F16;

/// A passive, constant-time state machine for decoding SMT160 timestamps.
///
/// The decoder does not perform I/O. Instead, it reacts to timestamps pushed into it.
/// This allows it to be used in high-frequency interrupts (RTIC) or async loops.
pub struct Smt160Decoder {
    last_rise: Option<u64>,
    last_fall: Option<u64>,
    stability_counter: u8,

    // History buffer for jitter filtering
    history: [I16F16; 2],
    history_len: u8,
    last_freq: Option<u32>,
}

impl Smt160Decoder {
    /// Creates a new decoder instance with empty history.
    pub fn new() -> Self {
        Self {
            last_rise: None,
            last_fall: None,
            stability_counter: 0,
            history: [I16F16::ZERO; 2],
            history_len: 0,
            last_freq: None,
        }
    }

    /// Resets the internal state, clearing history and stability counters.
    /// Use this after a known hardware error or when restarting the sensor.
    pub fn reset(&mut self) {
        self.last_rise = None;
        self.last_fall = None;
        self.stability_counter = 0;
        self.history_len = 0;
        self.last_freq = None;
    }

    /// Process a new edge timestamp (in microseconds).
    /// 
    /// # Arguments
    /// * `is_rising` - True if the edge is rising, false if falling.
    /// * `timestamp_us` - The time at which the edge occurred (µs).
    ///
    /// # Returns
    /// * `Ok(Some(temp))` - A valid, filtered temperature reading.
    /// * `Ok(None)` - Reading in progress or waiting for more pulses to stabilize.
    /// * `Err(Smt160Error)` - Validation failed (Jitter, Frequency, etc).
    pub fn push_edge(&mut self, is_rising: bool, timestamp_us: u64) -> Result<Option<I16F16>, Smt160Error> {
        if is_rising {
            if let (Some(prev_rise), Some(prev_fall)) = (self.last_rise, self.last_fall) {
                // Secure timing calculation: handle timer wraps by assuming the interval 
                // is within the 64-bit range.
                let period = timestamp_us.checked_sub(prev_rise).unwrap_or(0);
                let high_time = prev_fall.checked_sub(prev_rise).unwrap_or(0);

                // Prepare for the next cycle
                self.last_rise = Some(timestamp_us);

                if period == 0 || high_time == 0 || high_time > period {
                    self.reset();
                    return Err(Smt160Error::SequenceViolation);
                }

                // --- Frequency Validation ---
                let freq = 1_000_000 / period as u32;
                if freq < MIN_FREQ || freq > MAX_FREQ {
                    self.reset();
                    return Err(Smt160Error::FrequencyOutOfRange(freq));
                }

                // Frequency Watchdog: Ensure frequency doesn't shift >10% between pulses.
                if let Some(last_f) = self.last_freq {
                    if last_f.abs_diff(freq) > (last_f / 10) {
                        self.reset();
                        return Err(Smt160Error::SequenceViolation);
                    }
                }
                self.last_freq = Some(freq);

                // --- Fixed-Point Duty Cycle Calculation ---
                let period_fixed = I16F16::from_num(period);
                let high_fixed = I16F16::from_num(high_time);
                let dc = high_fixed / period_fixed;

                if dc < MIN_DC || dc > MAX_DC {
                    self.reset();
                    return Err(Smt160Error::InvalidDutyCycle(dc));
                }

                // Apply transfer function: T = (DC - 0.320) / 0.00470
                let temp = (dc - DC_OFFSET) / DC_STEP;

                // Thermal Bounds Check
                if temp < MIN_TEMP || temp > MAX_TEMP {
                    self.reset();
                    return Err(Smt160Error::ThermalOverload(temp));
                }

                // --- Jitter Filtering ---
                // Compare new reading with the average of the last two.
                if self.history_len == 2 {
                    let avg = (self.history[0] + self.history[1]) / I16F16::from_num(2);
                    let diff = if temp > avg { temp - avg } else { avg - temp };

                    if diff > MAX_JITTER {
                        self.reset();
                        return Err(Smt160Error::HighJitter);
                    }
                }

                // Update history buffer
                self.history[0] = self.history[1];
                self.history[1] = temp;
                if self.history_len < 2 {
                    self.history_len += 1;
                }

                // --- Stability Counter ---
                // Require 5 consecutive valid pulses after a reset before yielding a result.
                if self.stability_counter < 5 {
                    self.stability_counter += 1;
                    if self.stability_counter < 5 {
                        return Ok(None);
                    }
                }

                return Ok(Some(temp));
            } else {
                // First rise detected
                self.last_rise = Some(timestamp_us);
            }
        } else {
            // Processing Falling Edge
            if self.last_rise.is_some() {
                self.last_fall = Some(timestamp_us);
            } else {
                // Spurious fall detected without a preceding rise
                self.reset();
                return Err(Smt160Error::SequenceViolation);
            }
        }

        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normal_operation() {
        let mut decoder = Smt160Decoder::new();
        let period = 1000;
        let high = 438;
        let mut ts = 0;
        for _ in 0..6 {
            let _ = decoder.push_edge(true, ts);
            ts += high;
            let _ = decoder.push_edge(false, ts);
            ts += period - high;
        }
        let res = decoder.push_edge(true, ts).unwrap();
        assert!(res.is_some());
        let temp = res.unwrap();
        assert!(temp > 25.0 && temp < 25.2);
    }

    #[test]
    fn test_frequency_validation() {
        let mut decoder = Smt160Decoder::new();
        let _ = decoder.push_edge(true, 0);
        let _ = decoder.push_edge(false, 1000);
        let res = decoder.push_edge(true, 10000);
        assert!(matches!(res, Err(Smt160Error::FrequencyOutOfRange(_))));
    }

    #[test]
    fn test_jitter_filtering() {
        let mut decoder = Smt160Decoder::new();
        let mut ts = 0;
        for _ in 0..7 {
            let _ = decoder.push_edge(true, ts);
            ts += 438;
            let _ = decoder.push_edge(false, ts);
            ts += 562;
        }
        let _ = decoder.push_edge(true, ts);
        ts += 461;
        let _ = decoder.push_edge(false, ts);
        ts += 539;
        let res = decoder.push_edge(true, ts);
        assert!(matches!(res, Err(Smt160Error::HighJitter)));
    }
}