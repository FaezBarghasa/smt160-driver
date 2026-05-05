//! The high-precision SMT160 logic engine.

use crate::config::*;
use crate::Smt160Error;
use fixed::types::{I32F32, I16F16};

/// A passive, constant-time state machine for decoding SMT160 timestamps with high precision.
pub struct Smt160Decoder {
    last_rise: Option<u64>,
    last_fall: Option<u64>,
    buffer: [I16F16; 16],
    buf_idx: usize,
    buf_full: bool,
}

impl Smt160Decoder {
    /// Creates a new decoder instance with 16-sample buffer.
    pub const fn new() -> Self {
        Self {
            last_rise: None,
            last_fall: None,
            buffer: [I16F16::ZERO; 16],
            buf_idx: 0,
            buf_full: false,
        }
    }

    /// Resets the internal state.
    pub fn reset(&mut self) {
        self.last_rise = None;
        self.last_fall = None;
        self.buf_idx = 0;
        self.buf_full = false;
    }

    /// Process a new edge timestamp (in microseconds).
    pub fn push_edge(&mut self, is_rising: bool, timestamp_us: u64) -> Result<Option<I16F16>, Smt160Error> {
        if is_rising {
            if let (Some(prev_rise), Some(prev_fall)) = (self.last_rise, self.last_fall) {
                let active_time = prev_fall.wrapping_sub(prev_rise);
                let period = timestamp_us.wrapping_sub(prev_rise);

                // Update for next cycle
                self.last_rise = Some(timestamp_us);

                if period == 0 || active_time == 0 || active_time >= period {
                    return Err(Smt160Error::SequenceViolation);
                }

                // Frequency validation
                let freq = 1_000_000 / period as u32;
                if freq < MIN_FREQ || freq > MAX_FREQ {
                    return Err(Smt160Error::FrequencyOutOfRange);
                }

                // High-precision duty cycle and temperature calculation
                let duty_cycle = I32F32::from_num(active_time) / I32F32::from_num(period);
                
                // Typical physical bounds for SMT160 are 0.3 to 0.98
                if duty_cycle < I32F32::from_num(0.3) || duty_cycle > I32F32::from_num(0.98) {
                    return Err(Smt160Error::InvalidDutyCycle);
                }

                let temp_i32 = (duty_cycle - DC_OFFSET) * INV_STEP;
                let temp = I16F16::from_num(temp_i32);

                // Thermal bounds check
                if temp < MIN_TEMP || temp > MAX_TEMP {
                    return Err(Smt160Error::ThermalOverload);
                }

                // Outlier rejection and moving average
                if self.buf_full {
                    let avg = self.average();
                    let diff = if temp > avg { temp - avg } else { avg - temp };
                    
                    // Reject if > 2.0°C from rolling average
                    if diff > I16F16::from_num(2) {
                        return Err(Smt160Error::HighJitter);
                    }
                }

                // Add to circular buffer
                self.buffer[self.buf_idx] = temp;
                self.buf_idx = (self.buf_idx + 1) % 16;
                if self.buf_idx == 0 {
                    self.buf_full = true;
                }

                if self.buf_full {
                    Ok(Some(self.average()))
                } else {
                    Ok(None)
                }
            } else {
                self.last_rise = Some(timestamp_us);
                Ok(None)
            }
        } else {
            // Falling edge
            if self.last_rise.is_some() {
                self.last_fall = Some(timestamp_us);
            }
            Ok(None)
        }
    }

    fn average(&self) -> I16F16 {
        let len = if self.buf_full { 16 } else { self.buf_idx };
        if len == 0 {
            return I16F16::ZERO;
        }
        let mut sum = I32F32::ZERO;
        for i in 0..len {
            sum += I32F32::from_num(self.buffer[i]);
        }
        I16F16::from_num(sum / I32F32::from_num(len))
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
        for _ in 0..17 {
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
        let _ = decoder.push_edge(false, 100);
        let res = decoder.push_edge(true, 10000);
        assert!(matches!(res, Err(Smt160Error::FrequencyOutOfRange)));
    }

    #[test]
    fn test_outlier_rejection() {
        let mut decoder = Smt160Decoder::new();
        let mut ts = 0;
        for _ in 0..17 {
            let _ = decoder.push_edge(true, ts);
            ts += 438;
            let _ = decoder.push_edge(false, ts);
            ts += 562;
        }
        let _ = decoder.push_edge(true, ts);
        ts += 500;
        let _ = decoder.push_edge(false, ts);
        ts += 500;
        let res = decoder.push_edge(true, ts);
        assert!(matches!(res, Err(Smt160Error::HighJitter)));
    }
}