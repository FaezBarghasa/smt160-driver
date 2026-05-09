#![no_std]

//! # SMT160 High-Precision Industrial Driver
//!
//! This driver uses hardware-level DMA Burst and Timer Reset Mode to achieve 
//! absolute zero-jitter capture of SMT160 temperature sensor signals.

pub mod error;
pub mod types;
pub mod math;
pub mod telemetry;
pub mod hal;

pub use error::Smt160Error;
pub use types::{Uninitialized, Ready};
pub use math::SignalDecoder;
pub use telemetry::Smt160Status;

use fixed::types::I32F32;
use core::marker::PhantomData;
use crate::hal::Smt160Hal;

/// Configuration for the SMT160 driver, allowing tuning for different environments.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Jitter tolerance as a percentage (e.g., 0.005 for 0.5%).
    pub jitter_threshold_pct: I32F32,
}

impl Config {
    /// Recommended settings for industrial environments (low noise tolerance).
    pub fn industrial() -> Self {
        Self {
            jitter_threshold_pct: I32F32::from_num(0.005),
        }
    }

    /// Settings optimized for fast sampling in clean environments.
    pub fn fast() -> Self {
        Self {
            jitter_threshold_pct: I32F32::from_num(0.02),
        }
    }
}

/// The generic Smt160 driver, decoupled from hardware via the Smt160Hal trait.
pub struct Smt160Driver<H, S> {
    hal: H,
    _state: PhantomData<S>,
    config: Config,
    last_temp: Option<I32F32>,
    last_period: u32,
    sample_count: u32,
    watchdog_ticks: u32,
    pub status: Smt160Status,
}

impl<H> Smt160Driver<H, Uninitialized> 
where 
    H: Smt160Hal,
{
    /// Creates a new uninitialized driver instance.
    pub fn new(hal: H, config: Config) -> Self {
        Self {
            hal,
            _state: PhantomData,
            config,
            last_temp: None,
            last_period: 0,
            sample_count: 0,
            watchdog_ticks: 0,
            status: Smt160Status::empty(),
        }
    }

    /// Initializes the hardware and transitions to the `Ready` state.
    pub fn init(mut self, timer_freq: u32) -> Result<Smt160Driver<H, Ready>, Smt160Error> {
        self.hal.setup(timer_freq)?;
        
        Ok(Smt160Driver {
            hal: self.hal,
            _state: PhantomData,
            config: self.config,
            last_temp: None,
            last_period: 0,
            sample_count: 0,
            watchdog_ticks: 0,
            status: Smt160Status::empty(),
        })
    }
}

impl<H> Smt160Driver<H, Ready> 
where 
    H: Smt160Hal,
{
    /// Polls the hardware for new data and returns the filtered temperature.
    ///
    /// This method performs:
    /// 1. Jitter detection based on the `Config` threshold.
    /// 2. Raw signal decoding.
    /// 3. Adaptive EWMA filtering.
    pub fn read_temperature(&mut self) -> Option<I32F32> {
        if !self.hal.is_new_data_available() {
            self.tick_watchdog();
            return None;
        }

        self.watchdog_ticks = 0;
        self.status.remove(Smt160Status::SENSOR_TIMEOUT);

        let edge = self.hal.read_raw();
        
        // 1. Jitter Detection
        if self.last_period > 0 {
            let delta = if edge.period_ticks > self.last_period {
                edge.period_ticks - self.last_period
            } else {
                self.last_period - edge.period_ticks
            };
            
            let threshold = (I32F32::from_num(self.last_period) * self.config.jitter_threshold_pct).to_num::<u32>();
            
            if delta > threshold {
                self.status.insert(Smt160Status::JITTER_DETECTED);
            } else {
                self.status.remove(Smt160Status::JITTER_DETECTED);
            }
        }
        self.last_period = edge.period_ticks;

        // 2. Decode and Filter
        match SignalDecoder::decode(edge.period_ticks, edge.high_ticks) {
            Ok(raw) => {
                self.status.remove(Smt160Status::OUT_OF_BOUNDS);
                let corrected = SignalDecoder::apply_nlc(raw);
                
                let filtered = SignalDecoder::apply_adaptive_filter(
                    corrected,
                    self.last_temp,
                    self.sample_count
                );
                
                self.last_temp = Some(filtered);
                self.sample_count = self.sample_count.saturating_add(1);
                Some(filtered)
            }
            Err(_) => {
                self.status.insert(Smt160Status::OUT_OF_BOUNDS);
                None
            }
        }
    }

    /// Internal watchdog tick. Should be called if no data is available.
    fn tick_watchdog(&mut self) {
        self.watchdog_ticks = self.watchdog_ticks.saturating_add(1);
        if self.watchdog_ticks >= 500 { // Assuming ~10ms polling, this is 5s
            self.status.insert(Smt160Status::SENSOR_TIMEOUT);
        }
    }

    /// Returns the current hardware status flags.
    pub fn status(&self) -> Smt160Status {
        self.status
    }

    /// Returns the underlying HAL for direct access if needed.
    pub fn hal_mut(&mut self) -> &mut H {
        &mut self.hal
    }
}
