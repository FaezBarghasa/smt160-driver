//! The high-precision SMT160 logic engine.

use crate::config::*;
use crate::{Reading, Smt160Status, Smt160Error};
use fixed::types::{I16F16, I32F32};

/// A passive, constant-time state machine for decoding SMT160 timestamps with high precision.
/// 
/// # Hazards
/// - **Clock Mismatch**: If the configured `clock_mhz` does not match the actual timer frequency, 
///   the resulting temperature will have a linear error (e.g., 72MHz vs 36MHz results in ~50% error).
/// - **Interrupt Jitter**: High jitter in edge capture (e.g., software polling) will degrade precision. 
///   Use hardware Input Capture for optimal results.
/// 
/// # Performance
/// - **Calculations**: All math is performed using `I32F32` fixed-point arithmetic, suitable for 
///   Cortex-M cores without FPU.
/// - **Memory**: Uses a 16-sample buffer and a few state variables. No heap allocation.
pub struct Smt160Decoder {
    last_rise: Option<u64>,
    last_fall: Option<u64>,
    buffer: [I16F16; 16],
    buf_idx: usize,
    buf_full: bool,
    clock_mhz: u32,
    ewma: Option<I16F16>,
    sample_count: u32,
    last_edge_time: Option<u64>,
}

impl Smt160Decoder {
    /// Creates a new decoder instance assuming timestamps are in **microseconds** (1MHz clock).
    pub const fn new() -> Self {
        Self {
            last_rise: None,
            last_fall: None,
            buffer: [I16F16::ZERO; 16],
            buf_idx: 0,
            buf_full: false,
            clock_mhz: 1,
            ewma: None,
            sample_count: 0,
            last_edge_time: None,
        }
    }

    /// Creates a new decoder instance with a custom clock frequency in MHz.
    /// 
    /// Use this for high-precision capture (e.g., `72` for a 72MHz STM32 timer).
    pub const fn with_clock(mhz: u32) -> Self {
        debug_assert!(mhz > 0 && mhz <= 200, "Clock MHz must be between 1 and 200");
        let mut s = Self::new();
        s.clock_mhz = if mhz > 0 { mhz } else { 1 };
        s
    }

    /// Creates a new decoder instance from STM32F1xx RCC clocks.
    /// 
    /// # Logic
    /// The timer frequency calculation follows the STM32F1 reference manual:
    /// - If APB1 prescaler is 1, the timer frequency is equal to PCLK1.
    /// - If APB1 prescaler is NOT 1, the timer frequency is 2 * PCLK1.
    #[cfg(feature = "stm32f1")]
    pub fn from_clocks(clocks: &stm32f1xx_hal::rcc::Clocks) -> Self {
        let hclk_hz = clocks.hclk().to_Hz();
        let pclk1_hz = clocks.pclk1().to_Hz();
        
        let timer_freq = Self::calculate_timer_freq(hclk_hz, pclk1_hz);
        Self::with_clock(timer_freq)
    }

    /// Internal helper to calculate timer frequency from HCLK and PCLK1.
    pub fn calculate_timer_freq(hclk_hz: u32, pclk1_hz: u32) -> u32 {
        if pclk1_hz == 0 { return 1; }
        let prescaler = hclk_hz / pclk1_hz;
        
        if prescaler == 1 {
            pclk1_hz / 1_000_000
        } else {
            (pclk1_hz / 1_000_000) * 2
        }
    }

    /// Resets the internal state and circular buffer.
    pub fn reset(&mut self) {
        self.last_rise = None;
        self.last_fall = None;
        self.buf_idx = 0;
        self.buf_full = false;
        self.ewma = None;
        self.sample_count = 0;
        self.last_edge_time = None;
    }

    /// Process a new edge timestamp.
    /// 
    /// Returns `Ok(Some(Reading))` when a new filtered reading is ready.
    pub fn push_edge(&mut self, is_rising: bool, timestamp: u64) -> Result<Option<Reading>, Smt160Error> {
        let mut status = Smt160Status::OK;
        
        // Timeout check: 100ms detection
        if let Some(last) = self.last_edge_time {
            let elapsed_ticks = timestamp.wrapping_sub(last);
            // clock_mhz is MHz, so clock_mhz * 1000 is ticks per ms
            let elapsed_ms = elapsed_ticks / (self.clock_mhz as u64 * 1000);
            if elapsed_ms > 100 {
                status |= Smt160Status::SIGNAL_LOSS;
            }
        }
        self.last_edge_time = Some(timestamp);

        if is_rising {
            if let (Some(prev_rise), Some(prev_fall)) = (self.last_rise, self.last_fall) {
                let active_time = prev_fall.wrapping_sub(prev_rise);
                let period = timestamp.wrapping_sub(prev_rise);

                // Update for next cycle
                self.last_rise = Some(timestamp);

                if period == 0 || active_time == 0 || active_time >= period {
                    return Err(Smt160Error::SequenceViolation);
                }

                // Frequency validation (1kHz - 4kHz)
                let freq = (self.clock_mhz as u64 * 1_000_000) / period;
                if freq < MIN_FREQ as u64 || freq > MAX_FREQ as u64 {
                    status |= Smt160Status::FREQUENCY_ERROR;
                }

                // High-precision duty cycle calculation
                let duty_cycle = I32F32::from_num(active_time) / I32F32::from_num(period);
                
                // Physical boundary guardrails (0.320 to 0.980)
                if duty_cycle < I32F32::from_num(0.32) || duty_cycle > I32F32::from_num(0.98) {
                    status |= Smt160Status::BOUNDARY_VIOLATION;
                }

                // Temperature calculation: T = (DC - 0.320) * 212.77
                let temp_i32 = (duty_cycle - DC_OFFSET) * INV_STEP;
                let temp = I16F16::from_num(temp_i32);

                // Thermal bounds check
                if temp < MIN_TEMP || temp > MAX_TEMP {
                    status |= Smt160Status::OUT_OF_BOUNDS;
                }

                // Adaptive EWMA Filtering
                // Adaptivity: If abs(current - prev) > 5.0°C, increase alpha to 0.8
                let diff = if let Some(last_ewma) = self.ewma {
                    if temp > last_ewma { temp - last_ewma } else { last_ewma - temp }
                } else {
                    I16F16::ZERO
                };

                let alpha = if self.sample_count < 16 || diff > I16F16::from_num(5) {
                    I32F32::from_num(0.8)
                } else {
                    I32F32::from_num(0.1)
                };

                let current_ewma = if let Some(last_ewma) = self.ewma {
                    let val = alpha * I32F32::from_num(temp) + (I32F32::ONE - alpha) * I32F32::from_num(last_ewma);
                    I16F16::from_num(val)
                } else {
                    temp
                };
                self.ewma = Some(current_ewma);
                self.sample_count = self.sample_count.saturating_add(1);

                // Circular buffer for Jitter Warning detection
                let avg = self.average();
                if self.buf_full {
                    let jitter = if temp > avg { temp - avg } else { avg - temp };
                    if jitter > I16F16::from_num(2) {
                        status |= Smt160Status::JITTER_ALERT;
                    }
                }

                self.buffer[self.buf_idx] = temp;
                self.buf_idx = (self.buf_idx + 1) % 16;
                if self.buf_idx == 0 {
                    self.buf_full = true;
                }

                Ok(Some(Reading {
                    value: current_ewma,
                    status,
                }))
            } else {
                self.last_rise = Some(timestamp);
                Ok(None)
            }
        } else {
            // Falling edge
            if self.last_rise.is_some() {
                self.last_fall = Some(timestamp);
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
        let reading = res.unwrap();
        assert!(reading.value > 25.0 && reading.value < 25.2);
        assert_eq!(reading.status, Smt160Status::OK);
    }

    #[test]
    fn test_frequency_validation() {
        let mut decoder = Smt160Decoder::new();
        let _ = decoder.push_edge(true, 0);
        let _ = decoder.push_edge(false, 100);
        let res = decoder.push_edge(true, 10000).unwrap();
        assert!(res.is_some());
        assert!(res.unwrap().status.contains(Smt160Status::FREQUENCY_ERROR));
    }

    #[test]
    fn test_boundary_violation() {
        let mut decoder = Smt160Decoder::new();
        let _ = decoder.push_edge(true, 0);
        let _ = decoder.push_edge(false, 100); // 10% DC (Invalid)
        let res = decoder.push_edge(true, 1000).unwrap();
        assert!(res.is_some());
        let reading = res.unwrap();
        assert!(reading.status.contains(Smt160Status::BOUNDARY_VIOLATION));
    }

    #[test]
    fn test_timer_frequency_calculation() {
        // Case 1: 72MHz System, APB1 Prescaler = 2 (PCLK1 = 36MHz)
        // Timer should be PCLK1 * 2 = 72MHz
        let hclk = 72_000_000;
        let pclk1 = 36_000_000;
        assert_eq!(Smt160Decoder::calculate_timer_freq(hclk, pclk1), 72);

        // Case 2: 72MHz System, APB1 Prescaler = 1 (PCLK1 = 72MHz) - Unusual for F1 but possible
        // Timer should be PCLK1 = 72MHz
        let pclk1_fast = 72_000_000;
        assert_eq!(Smt160Decoder::calculate_timer_freq(hclk, pclk1_fast), 72);

        // Case 3: 36MHz System, APB1 Prescaler = 1 (PCLK1 = 36MHz)
        // Timer should be PCLK1 = 36MHz
        let hclk_slow = 36_000_000;
        let pclk1_slow = 36_000_000;
        assert_eq!(Smt160Decoder::calculate_timer_freq(hclk_slow, pclk1_slow), 36);
    }

    #[test]
    fn test_thermal_step() {
        let mut decoder = Smt160Decoder::with_clock(1);
        let mut ts = 0;
        
        // Steady state at 25°C (DC = 0.4375)
        let period = 1000;
        let high = 438;
        
        for _ in 0..20 {
            let _ = decoder.push_edge(true, ts);
            let _ = decoder.push_edge(false, ts + high);
            ts += period;
        }
        
        let last = decoder.push_edge(true, ts).unwrap().unwrap();
        assert!(last.value > 24.5 && last.value < 25.5);
        
        // Step to 50°C (DC = 0.555)
        // DC = 0.320 + 0.00470 * 50 = 0.320 + 0.235 = 0.555
        let high_new = 555;
        
        // First sample after step should trigger adaptive alpha (0.8)
        let _ = decoder.push_edge(true, ts);
        let _ = decoder.push_edge(false, ts + high_new);
        ts += period;
        
        let res = decoder.push_edge(true, ts).unwrap().unwrap();
        // With alpha 0.8: 25.0 * 0.2 + 50.0 * 0.8 = 5.0 + 40.0 = 45.0
        assert!(res.value > 44.0, "Expected fast response due to adaptive alpha, got {}", res.value);
    }
}
