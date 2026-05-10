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
pub mod calibration;

pub use error::Smt160Error;
pub use types::{Uninitialized, Ready};
pub use math::SignalDecoder;
pub use telemetry::{Smt160Status, Diagnostics};
pub use calibration::{Calibration, LinearCalibration};

use fixed::types::I32F32;
use core::marker::PhantomData;
use crate::hal::Smt160Hal;

/// Configuration for the SMT160 driver, allowing tuning for different environments.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Config {
    /// Jitter tolerance as a percentage (e.g., 0.005 for 0.5%).
    pub jitter_threshold_pct: I32F32,
    /// Timeout in milliseconds.
    pub timeout_ms: u32,
}

impl Config {
    /// Recommended settings for industrial environments (low noise tolerance).
    pub fn industrial() -> Self {
        Self {
            jitter_threshold_pct: I32F32::from_num(0.005),
            timeout_ms: 500,
        }
    }

    /// Settings optimized for fast sampling in clean environments.
    pub fn fast() -> Self {
        Self {
            jitter_threshold_pct: I32F32::from_num(0.02),
            timeout_ms: 100,
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
    last_update: fugit::TimerInstantU32<1000>,
    pub status: Smt160Status,
    pub diagnostics: Diagnostics,
    pub calibration: LinearCalibration,
    pub nlc_table: Option<&'static [(I32F32, I32F32)]>,
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
            last_update: fugit::TimerInstantU32::from_ticks(0),
            status: Smt160Status::empty(),
            diagnostics: Diagnostics::new(),
            calibration: LinearCalibration::default(),
            nlc_table: None,
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
            last_update: fugit::TimerInstantU32::from_ticks(0),
            status: Smt160Status::empty(),
            diagnostics: self.diagnostics,
            calibration: self.calibration,
            nlc_table: self.nlc_table,
        })
    }
}

impl<H> Smt160Driver<H, Ready> 
where 
    H: Smt160Hal,
{
    /// Re-initializes the hardware and resets internal driver state.
    ///
    /// This is useful for autonomous recovery after a sensor timeout or signal loss.
    pub fn reinit(&mut self, timer_freq: u32) -> Result<(), Smt160Error> {
        self.hal.setup(timer_freq)?;
        self.status = Smt160Status::empty();
        self.last_temp = None;
        self.sample_count = 0;
        self.last_period = 0;
        self.last_update = fugit::TimerInstantU32::from_ticks(0);
        Ok(())
    }

    /// Polls the hardware for new data and returns the filtered temperature.
    ///
    /// This method uses the provided monotonic to update the internal watchdog.
    pub fn read_temperature<M>(&mut self) -> Option<I32F32> 
    where 
        M: rtic_monotonics::Monotonic<Instant = fugit::TimerInstantU32<1000>>
    {
        let now = M::now();
        let elapsed = now.checked_duration_since(self.last_update)
            .unwrap_or(fugit::TimerDurationU32::from_ticks(0));

        if !self.hal.is_new_data_available() {
            if elapsed.to_millis() >= self.config.timeout_ms {
                self.status.insert(Smt160Status::SENSOR_TIMEOUT);
            }
            return None;
        }

        self.last_update = now;
        self.status.remove(Smt160Status::SENSOR_TIMEOUT);

        let edge = self.hal.read_raw();
        self.last_period = edge.period_ticks;
        self.diagnostics.update(edge.period_ticks);

        // Jitter Analysis: if sigma > 1.5% of mean period, set SIGNAL_NOISY
        let sigma = self.diagnostics.std_dev();
        let mean = self.diagnostics.mean_ticks;
        if mean > 0 && sigma > (mean * I32F32::from_num(0.015)) {
            self.status.insert(Smt160Status::SIGNAL_NOISY);
        } else {
            self.status.remove(Smt160Status::SIGNAL_NOISY);
        }

        // 2. Decode and Filter
        match SignalDecoder::decode(edge.period_ticks, edge.high_ticks) {
            Ok(raw) => {
                self.status.remove(Smt160Status::OUT_OF_BOUNDS);
                
                // Apply NLC (Custom or Default)
                let corrected = if let Some(table) = self.nlc_table {
                    SignalDecoder::apply_nlc_custom(raw, table)
                } else {
                    SignalDecoder::apply_nlc(raw)
                };
                
                // Apply Calibration
                let calibrated = self.calibration.calibrate(corrected);
                
                let filtered = SignalDecoder::apply_adaptive_filter(
                    calibrated,
                    self.last_temp,
                    self.sample_count
                );
                
                // Gradient Monitoring: |T_current - T_previous| / dt > 10.0 °C/s
                if let Some(prev) = self.last_temp {
                    let dt = elapsed.to_millis() as f32 / 1000.0;
                    if dt > 0.0 {
                        let gradient = (filtered - prev).abs() / I32F32::from_num(dt);
                        if gradient > 10.0 {
                            self.status.insert(Smt160Status::GRADIENT_ERROR);
                        } else {
                            self.status.remove(Smt160Status::GRADIENT_ERROR);
                        }
                    }
                }

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

    /// Asynchronously waits for a new data update from the hardware.
    pub async fn wait_for_update(&mut self) -> Result<(), Smt160Error> {
        self.hal.wait_for_new_data().await
    }

    /// Asynchronously waits for a new sample and returns the filtered temperature.
    pub async fn read_temp<M>(&mut self) -> Option<I32F32> 
    where 
        M: rtic_monotonics::Monotonic<Instant = fugit::TimerInstantU32<1000>>
    {
        if self.wait_for_update().await.is_ok() {
            self.read_temperature::<M>()
        } else {
            None
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
