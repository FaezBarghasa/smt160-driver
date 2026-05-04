use crate::config::*;
use crate::Smt160Error;
use fixed::macros::fixed;
use fixed::types::I16F16;

/// A passive, constant-time state machine for decoding SMT160 timestamps.
pub struct Smt160Decoder {
    last_rise: Option<u64>,
    last_fall: Option<u64>,
    stability_counter: u8,

    // History buffer for jitter filtering (Phase 4)
    history: [I16F16; 2],
    history_len: u8,
    last_freq: Option<u32>,
}

impl Smt160Decoder {
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

    pub fn reset(&mut self) {
        self.last_rise = None;
        self.last_fall = None;
        self.stability_counter = 0;
        self.history_len = 0;
        self.last_freq = None;
    }

    /// Process a new edge timestamp (in microseconds).
    /// Returns `Ok(Some(temp))` only when a stable, valid reading is acquired.
    pub fn push_edge(&mut self, is_rising: bool, timestamp_us: u64) -> Result<Option<I16F16>, Smt160Error> {
        if is_rising {
            if let (Some(prev_rise), Some(prev_fall)) = (self.last_rise, self.last_fall) {
                // Calculate timing securely with checked_sub to handle wrap-around gracefully
                let period = timestamp_us.checked_sub(prev_rise).unwrap_or(0);
                let high_time = prev_fall.checked_sub(prev_rise).unwrap_or(0);

                // Prepare for next cycle
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

                // Frequency Watchdog (max 10% shift)
                if let Some(last_f) = self.last_freq {
                    if last_f.abs_diff(freq) > (last_f / 10) {
                        self.reset();
                        return Err(Smt160Error::SequenceViolation);
                    }
                }
                self.last_freq = Some(freq);

                // --- Fixed-Point Math & Conversion ---
                let period_fixed = I16F16::from_num(period);
                let high_fixed = I16F16::from_num(high_time);
                let dc = high_fixed / period_fixed;

                if dc < MIN_DC || dc > MAX_DC {
                    self.reset();
                    return Err(Smt160Error::InvalidDutyCycle(dc));
                }

                let temp = (dc - DC_OFFSET) / DC_STEP;

                if temp < MIN_TEMP || temp > MAX_TEMP {
                    self.reset();
                    return Err(Smt160Error::ThermalOverload(temp));
                }

                // --- Jitter Filtering (Phase 4) ---
                if self.history_len == 2 {
                    let avg = (self.history[0] + self.history[1]) / fixed!(2.0: I16F16);
                    let diff = if temp > avg { temp - avg } else { avg - temp };

                    if diff > MAX_JITTER {
                        self.reset();
                        return Err(Smt160Error::HighJitter);
                    }
                }

                // Shift history buffer
                self.history[0] = self.history[1];
                self.history[1] = temp;
                if self.history_len < 2 {
                    self.history_len += 1;
                }

                // --- Stability Counter (Phase 4) ---
                // Require exactly 5 consecutive valid pulses before yielding the first reading
                if self.stability_counter < 5 {
                    self.stability_counter += 1;
                    if self.stability_counter < 5 {
                        return Ok(None);
                    }
                }

                return Ok(Some(temp));
            } else {
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