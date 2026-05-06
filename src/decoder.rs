//! High-Precision SMT160 Signal Logic Engine.

use crate::config::*;
use crate::{Reading, Smt160Status, Smt160Error};
use crate::math::apply_ewma_filter;
use fixed::types::{I16F16, I32F32};

/// A high-integrity state machine for decoding SMT160 pulse trains with deterministic performance.
/// 
/// # Architecture
/// This decoder is entirely passive and platform-agnostic. It accepts raw timer timestamps 
/// or ticks and performs all calculations using fixed-point arithmetic to ensure consistency 
/// across systems without an FPU.
pub struct Smt160Decoder {
    last_rising_edge_ticks: Option<u64>,
    last_falling_edge_ticks: Option<u64>,
    circular_buffer: [I16F16; 16],
    buffer_index: usize,
    is_buffer_full: bool,
    timer_clock_megahertz: u32,
    exponential_moving_average: Option<I16F16>,
    total_processed_samples: u32,
    last_captured_edge_time: Option<u64>,
}


impl Default for Smt160Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Smt160Decoder {
    /// Creates a new decoder instance with a default 1MHz (microsecond) clock resolution.
    pub const fn new() -> Self {
        Self {
            last_rising_edge_ticks: None,
            last_falling_edge_ticks: None,
            circular_buffer: [I16F16::ZERO; 16],
            buffer_index: 0,
            is_buffer_full: false,
            timer_clock_megahertz: 1,
            exponential_moving_average: None,
            total_processed_samples: 0,
            last_captured_edge_time: None,
        }
    }

    /// Creates a new decoder instance optimized for a specific hardware timer frequency.
    pub const fn new_standalone(timer_clock_megahertz: u32) -> Self {
        let mut decoder = Self::new();
        decoder.timer_clock_megahertz = if timer_clock_megahertz > 0 { timer_clock_megahertz } else { 1 };
        decoder
    }

    /// Resets all internal state, including filters and edge tracking.
    pub fn reset_state(&mut self) {
        self.last_rising_edge_ticks = None;
        self.last_falling_edge_ticks = None;
        self.buffer_index = 0;
        self.is_buffer_full = false;
        self.exponential_moving_average = None;
        self.total_processed_samples = 0;
        self.last_captured_edge_time = None;
    }

    /// Decodes a batch of raw capture words (Period/Active ticks) typically from a DMA buffer.
    /// 
    /// # Summary
    /// Iterates through the batch and returns the latest filtered reading.
    pub fn process_batch(
        &mut self, 
        dma_capture_data: &[u32], 
        duty_cycle_offset: I32F32, 
        inverse_step_constant: I32F32
    ) -> Result<Option<Reading>, Smt160Error> {
        let mut latest_reading = None;
        for &capture_word in dma_capture_data {
            let (period_ticks, active_ticks) = crate::conversion::unpack_dma_capture(capture_word);
            if period_ticks > 0 {
                latest_reading = Some(self.process_raw_ticks(
                    period_ticks, 
                    active_ticks, 
                    duty_cycle_offset, 
                    inverse_step_constant
                )?);
            }
        }
        Ok(latest_reading)
    }

    /// Processes a single cycle defined by its raw period and active ticks.
    /// 
    /// # Summary
    /// This is the core logic engine that validates signal integrity and calculates 
    /// the filtered temperature reading.
    pub fn process_raw_ticks(
        &mut self, 
        period_ticks: u64, 
        active_ticks: u64, 
        duty_cycle_offset: I32F32, 
        inverse_step_constant: I32F32
    ) -> Result<Reading, Smt160Error> {
        let mut operational_status = Smt160Status::OK;

        if period_ticks == 0 || active_ticks == 0 || active_ticks >= period_ticks {
            return Err(Smt160Error::SequenceViolation);
        }

        // Frequency validation (Industrial Range: 1kHz - 4kHz)
        let frequency_hz = (self.timer_clock_megahertz as u64 * 1_000_000)
            .checked_div(period_ticks)
            .ok_or(Smt160Error::InvalidSignal)?;
            
        if frequency_hz < MINIMUM_FREQUENCY_HZ as u64 || frequency_hz > MAXIMUM_FREQUENCY_HZ as u64 {
            operational_status |= Smt160Status::FREQUENCY_ERROR;
        }

        // High-precision duty cycle calculation (I32F32)
        let current_duty_cycle = I32F32::from_num(active_ticks)
            .checked_div(I32F32::from_num(period_ticks))
            .ok_or(Smt160Error::InvalidSignal)?;
        
        // Physical boundary validation (SMT160 Specs: 0.320 to 0.980)
        if current_duty_cycle < I32F32::from_num(0.32) || current_duty_cycle > I32F32::from_num(0.98) {
            operational_status |= Smt160Status::BOUNDARY_VIOLATION;
        }

        // Temperature calculation: T = (DutyCycle - Offset) * InverseStep
        let calculated_temperature_celsius = I16F16::from_num((current_duty_cycle - duty_cycle_offset) * inverse_step_constant);

        // Industrial thermal safety bounds check
        if calculated_temperature_celsius < MINIMUM_TEMPERATURE_CELSIUS || calculated_temperature_celsius > MAXIMUM_TEMPERATURE_CELSIUS {
            operational_status |= Smt160Status::OUT_OF_BOUNDS;
        }

        // Adaptive EWMA Filtering Logic
        let temperature_deviation = if let Some(previous_average) = self.exponential_moving_average {
            (calculated_temperature_celsius - previous_average).abs()
        } else {
            I16F16::ZERO
        };

        // Increase response speed if high deviation detected (>5.0°C) or during startup
        let smoothing_factor = if self.total_processed_samples < 16 || temperature_deviation > I16F16::from_num(5) {
            I32F32::from_num(0.8) // Fast tracking
        } else {
            I32F32::from_num(0.1) // Noise rejection
        };

        let updated_average = if let Some(previous_average) = self.exponential_moving_average {
            apply_ewma_filter(previous_average, calculated_temperature_celsius, smoothing_factor)
        } else {
            calculated_temperature_celsius
        };
        
        self.exponential_moving_average = Some(updated_average);
        self.total_processed_samples = self.total_processed_samples.saturating_add(1);

        // Signal Jitter Analysis
        let rolling_average = self.calculate_buffer_average();
        if self.is_buffer_full {
            let current_jitter = (calculated_temperature_celsius - rolling_average).abs();
            if current_jitter > I16F16::from_num(2) {
                operational_status |= Smt160Status::JITTER_ALERT;
            }
        }

        // Update circular buffer
        self.circular_buffer[self.buffer_index] = calculated_temperature_celsius;
        self.buffer_index = (self.buffer_index + 1) % 16;
        if self.buffer_index == 0 {
            self.is_buffer_full = true;
        }

        Ok(Reading {
            temperature_celsius: updated_average,
            status: operational_status,
        })
    }

    /// Pushes a new edge timestamp into the state machine.
    /// 
    /// # Summary
    /// Automatically manages Rising/Falling edge logic and returns a filtered reading 
    /// when a complete PWM cycle has been captured.
    pub fn push_edge_timestamp(
        &mut self, 
        is_rising_edge: bool, 
        capture_timestamp_ticks: u64, 
        duty_cycle_offset: I32F32, 
        inverse_step_constant: I32F32
    ) -> Result<Option<Reading>, Smt160Error> {
        self.last_captured_edge_time = Some(capture_timestamp_ticks);

        if is_rising_edge {
            if let (Some(prev_rise), Some(prev_fall)) = (self.last_rising_edge_ticks, self.last_falling_edge_ticks) {
                let active_ticks = prev_fall.wrapping_sub(prev_rise);
                let period_ticks = capture_timestamp_ticks.wrapping_sub(prev_rise);
                self.last_rising_edge_ticks = Some(capture_timestamp_ticks);
                
                Ok(Some(self.process_raw_ticks(
                    period_ticks, 
                    active_ticks, 
                    duty_cycle_offset, 
                    inverse_step_constant
                )?))
            } else {
                self.last_rising_edge_ticks = Some(capture_timestamp_ticks);
                Ok(None)
            }
        } else {
            if self.last_rising_edge_ticks.is_some() {
                self.last_falling_edge_ticks = Some(capture_timestamp_ticks);
            }
            Ok(None)
        }
    }

    fn calculate_buffer_average(&self) -> I16F16 {
        let sample_count = if self.is_buffer_full { 16 } else { self.buffer_index };
        if sample_count == 0 {
            return I16F16::ZERO;
        }
        let mut total_sum = I32F32::ZERO;
        for i in 0..sample_count {
            total_sum += I32F32::from_num(self.circular_buffer[i]);
        }
        I16F16::from_num(total_sum.checked_div(I32F32::from_num(sample_count)).unwrap_or(I32F32::ZERO))
    }
}
