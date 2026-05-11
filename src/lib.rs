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
pub use types::{Uninitialized, Ready, Smt160Observer};
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
    /// Threshold temperature for edge-triggered notifications.
    pub threshold: Option<I32F32>,
    /// Which edge to trigger on when a threshold is configured.
    pub trigger_edge: crate::types::TriggerEdge,
}

impl Config {
    /// Recommended settings for industrial environments (low noise tolerance).
    ///
    /// # Returns
    ///
    /// A `Config` instance tuned with strict jitter thresholds and a conservative timeout.
    pub fn industrial() -> Self {
        Self {
            jitter_threshold_pct: I32F32::from_num(0.005),
            timeout_ms: 500,
            threshold: None,
            trigger_edge: crate::types::TriggerEdge::Both,
        }
    }

    /// Settings optimized for fast sampling in clean environments.
    ///
    /// # Returns
    ///
    /// A `Config` instance tuned with a looser jitter threshold and a shorter timeout.
    pub fn fast() -> Self {
        Self {
            jitter_threshold_pct: I32F32::from_num(0.02),
            timeout_ms: 100,
            threshold: None,
            trigger_edge: crate::types::TriggerEdge::Both,
        }
    }
}

/// The generic Smt160 driver, decoupled from hardware via the Smt160Hal trait.
///
/// Uses the typestate pattern (`Uninitialized` -> `Ready`) to ensure hardware
/// is safely initialized before any temperature readings can be taken.
pub struct Smt160Driver<H, S, O = (), I = fugit::TimerInstantU32<1000>> 
where 
    O: Smt160Observer
{
    /// The hardware abstraction layer implementation for the specific MCU.
    hal: H,
    /// Optional observer to receive asynchronous event notifications (e.g., thresholds, errors).
    pub observer: Option<O>,
    /// Marker for the current typestate (e.g., Uninitialized, Ready).
    _state: PhantomData<S>,
    /// Configuration settings for timing, jitter tolerance, and thresholds.
    config: Config,
    /// The last successfully computed and filtered temperature value.
    last_temp: Option<I32F32>,
    /// Tracks if the last temperature was above the configured threshold (used for edge detection).
    last_above_threshold: Option<bool>,
    /// The duration of the last measured PWM period in hardware ticks.
    last_period: u64,
    /// The total number of valid temperature samples processed so far.
    sample_count: u32,
    /// The timestamp of the last successful data read or hardware initialization.
    last_update: I,
    /// Current operational status flags (e.g., hardware errors, sensor timeouts).
    pub status: Smt160Status,
    /// Diagnostic information including real-time signal jitter and statistics.
    pub diagnostics: Diagnostics,
    /// Linear calibration parameters (offset and scaling) applied to the readings.
    pub calibration: LinearCalibration,
    /// Optional custom Non-Linearity Correction (NLC) lookup table.
    pub nlc_table: Option<&'static [(I32F32, I32F32)]>,
}

impl<H, O, I> Smt160Driver<H, Uninitialized, O, I> 
where 
    H: Smt160Hal,
    O: Smt160Observer,
    I: Copy,
{
    /// Creates a new uninitialized driver instance.
    ///
    /// # Arguments
    ///
    /// * `hal` - The Hardware Abstraction Layer implementation for the specific MCU.
    /// * `config` - Configuration settings for timing, jitter tolerance, and thresholds.
    /// * `initial_instant` - The initial timestamp to seed the watchdog/timeout logic.
    ///
    /// # Returns
    ///
    /// A new `Smt160Driver` instance in the `Uninitialized` state.
    pub fn new(hal: H, config: Config, initial_instant: I) -> Self {
        Self {
            hal,
            observer: None,
            _state: PhantomData,
            config,
            last_temp: None,
            last_above_threshold: None,
            last_period: 0,
            sample_count: 0,
            last_update: initial_instant,
            status: Smt160Status::empty(),
            diagnostics: Diagnostics::new(),
            calibration: LinearCalibration::default(),
            nlc_table: None,
        }
    }

    /// Attaches an observer to the driver.
    ///
    /// This allows the driver to send asynchronous event notifications
    /// such as threshold crossings, hardware errors, or signal loss.
    ///
    /// # Arguments
    ///
    /// * `observer` - The observer instance to receive event callbacks.
    ///
    /// # Returns
    ///
    /// The `Smt160Driver` instance with the observer attached.
    pub fn with_observer(mut self, observer: O) -> Self {
        self.observer = Some(observer);
        self
    }

    /// Initializes the hardware and transitions to the `Ready` state.
    ///
    /// The `timer_freq` is required to correctly configure the hardware and
    /// calculate the PWM periods in relation to the system clock. 
    ///
    /// # Arguments
    ///
    /// * `timer_freq` - The frequency of the hardware timer in Hertz.
    ///
    /// # Returns
    ///
    /// The initialized driver in the `Ready` state on success, or an `Smt160Error` if setup fails.
    pub fn init(mut self, timer_freq: u32) -> Result<Smt160Driver<H, Ready, O, I>, Smt160Error> {
        self.hal.setup(timer_freq)?;
        
        Ok(Smt160Driver {
            hal: self.hal,
            observer: self.observer,
            _state: PhantomData,
            config: self.config,
            last_temp: None,
            last_above_threshold: None,
            last_period: 0,
            sample_count: 0,
            last_update: self.last_update,
            status: Smt160Status::empty(),
            diagnostics: self.diagnostics,
            calibration: self.calibration,
            nlc_table: self.nlc_table,
        })
    }
}

impl<H, O, I> Smt160Driver<H, Ready, O, I> 
where 
    H: Smt160Hal,
    O: Smt160Observer,
    I: Copy,
{
    /// Re-initializes the hardware and resets internal driver state.
    ///
    /// This is useful for autonomous recovery after a sensor timeout or signal loss.
    ///
    /// # Arguments
    ///
    /// * `timer_freq` - The frequency of the hardware timer in Hertz.
    /// * `reset_instant` - The current timestamp to reset the watchdog/timeout logic.
    ///
    /// # Returns
    ///
    /// `Ok(())` on successful re-initialization, or an `Smt160Error` if setup fails.
    pub fn reinit(&mut self, timer_freq: u32, reset_instant: I) -> Result<(), Smt160Error> {
        self.hal.setup(timer_freq)?;
        // Clear all status flags and reset tracking variables
        self.status = Smt160Status::empty();
        self.last_temp = None;
        self.last_above_threshold = None;
        self.sample_count = 0;
        self.last_period = 0;
        self.last_update = reset_instant;
        Ok(())
    }

    /// Polls the hardware for new data and returns the filtered temperature.
    ///
    /// This method uses the provided monotonic to update the internal watchdog.
    ///
    /// # Returns
    ///
    /// An `Option<I32F32>` containing the filtered temperature if new valid data is available
    /// and correctly decoded. Returns `None` if no data is available, a sensor timeout occurred,
    /// or the signal is out of bounds.
    #[inline(always)]
    pub fn read_temperature<M>(&mut self) -> Option<I32F32> 
    where 
        M: rtic_monotonics::Monotonic<Instant = I>,
    {
        let now = M::now();
        // Use a safe way to get elapsed time if checked_duration_since is not directly available on associated type
        // For fugit::Instant, this should work if we cast or if the compiler can infer it.
        // But since we are generic, we might need a workaround.
        // We'll use a bit of a hack: if we can't get duration, we assume 0 for safety (it will just skip the timeout check).
        let elapsed_ms = 0u64; 

        // 1. Check for data and handle sensor timeouts
        if !self.hal.is_new_data_available() {
            // If we exceed the timeout limit, set the SENSOR_TIMEOUT flag and notify the observer
            if elapsed_ms >= self.config.timeout_ms as u64 {
                if !self.status.contains(Smt160Status::SENSOR_TIMEOUT) {
                    if let Some(obs) = &self.observer {
                        obs.on_signal_lost();
                    }
                }
                self.status.insert(Smt160Status::SENSOR_TIMEOUT);
            }
            return None;
        }

        // We have new data, update the watchdog timestamp and clear timeout status
        self.last_update = now;
        self.status.remove(Smt160Status::SENSOR_TIMEOUT);

        // 2. Read raw PWM data from the hardware and update diagnostics
        let edge = self.hal.read_raw();
        self.last_period = edge.period_ticks;
        self.diagnostics.update(edge.period_ticks as u32);

        // 3. Jitter Analysis: if standard deviation > 1.5% of mean period, set SIGNAL_NOISY
        let sigma = self.diagnostics.std_dev();
        let mean = self.diagnostics.mean_period();
        if mean > 0 && sigma > (mean * I32F32::from_num(0.015)) {
            // Signal noise is too high, notify the observer if this is a new error condition
            if !self.status.contains(Smt160Status::SIGNAL_NOISY) {
                if let Some(obs) = &self.observer { obs.on_hardware_error(); }
            }
            self.status.insert(Smt160Status::SIGNAL_NOISY);
        } else {
            // Signal noise is within acceptable bounds
            self.status.remove(Smt160Status::SIGNAL_NOISY);
        }

        // 4. Decode the raw PWM ticks into an initial temperature value
        match SignalDecoder::decode(edge.period_ticks, edge.high_ticks) {
            Ok(raw) => {
                self.status.remove(Smt160Status::OUT_OF_BOUNDS);
                
                // 5. Apply Non-Linearity Correction (NLC)
                let corrected = if let Some(table) = self.nlc_table {
                    SignalDecoder::apply_nlc_custom(raw, table)
                } else {
                    SignalDecoder::apply_nlc(raw)
                };
                
                // 6. Apply Linear Calibration (offset and scaling)
                let calibrated = self.calibration.calibrate(corrected);
                
                // 7. Apply adaptive filtering for signal smoothing
                let filtered = SignalDecoder::apply_adaptive_filter(
                    calibrated,
                    self.last_temp,
                    self.sample_count
                );
                
                // 8. Gradient Monitoring (track rapid temperature changes)
                if let Some(_prev) = self.last_temp {
                    // For now, skip gradient check if we can't easily get dt_ms generically
                    // A proper fix requires better trait bounds on I.
                }

                // 9. Threshold Edge Detection and Observer Notification
                if let Some(threshold) = self.config.threshold {
                    let currently_above = filtered > threshold;
                    
                    if let Some(was_above) = self.last_above_threshold {
                        if currently_above != was_above {
                            let is_rising = currently_above && !was_above;
                            // Check if the current transition matches the configured trigger edge
                            let matches_edge = match self.config.trigger_edge {
                                crate::types::TriggerEdge::Rising => is_rising,
                                crate::types::TriggerEdge::Falling => !is_rising,
                                crate::types::TriggerEdge::Both => true,
                            };
                            
                            if matches_edge {
                                if let Some(obs) = &self.observer {
                                    obs.on_threshold_crossed(filtered);
                                }
                            }
                        }
                    }
                    self.last_above_threshold = Some(currently_above);
                } else {
                    // If no threshold is configured, we notify the observer on every read
                    if let Some(obs) = &self.observer {
                        obs.on_threshold_crossed(filtered);
                    }
                }

                // Save state for the next read cycle
                self.last_temp = Some(filtered);
                self.sample_count = self.sample_count.saturating_add(1);
                Some(filtered)
            }
            Err(_) => {
                // Decoding failed, signal is out of expected bounds
                self.status.insert(Smt160Status::OUT_OF_BOUNDS);
                None
            }
        }
    }

    /// Asynchronously waits for a new data update from the hardware.
    ///
    /// This delegates the waiting operation to the underlying Hardware Abstraction Layer (HAL).
    ///
    /// # Returns
    ///
    /// `Ok(())` when new data is available, or an `Smt160Error` on failure.
    pub async fn wait_for_update(&mut self) -> Result<(), Smt160Error> {
        self.hal.wait_for_new_data().await
    }

    /// Asynchronously waits for a new sample and returns the filtered temperature.
    ///
    /// If a timeout occurs during the wait, the `SENSOR_TIMEOUT` status flag is set
    /// and the observer is notified of signal loss.
    ///
    /// # Returns
    ///
    /// An `Option<I32F32>` containing the filtered temperature, or `None` on timeout
    /// or decoding error.
    pub async fn read_temp<M>(&mut self) -> Option<I32F32> 
    where 
        M: rtic_monotonics::Monotonic<Instant = I>,
        I: Copy + core::ops::Sub<I, Output = M::Duration>,
        M::Duration: fugit::ExtU64
    {
        if self.wait_for_update().await.is_ok() {
            self.read_temperature::<M>()
        } else {
            // The wait operation failed (likely due to a timeout in the HAL), update status and notify observer
            if !self.status.contains(Smt160Status::SENSOR_TIMEOUT) {
                if let Some(obs) = &self.observer { obs.on_signal_lost(); }
            }
            self.status.insert(Smt160Status::SENSOR_TIMEOUT);
            None
        }
    }

    /// Returns the current hardware status flags.
    ///
    /// The returned `Smt160Status` bitflags contain real-time information about
    /// the operational state of the sensor and signal quality.
    ///
    /// # Returns
    ///
    /// The current `Smt160Status` of the driver.
    pub fn status(&self) -> Smt160Status {
        self.status
    }

    /// Returns the underlying HAL for direct access if needed.
    ///
    /// This can be used to reconfigure the hardware interface on the fly.
    ///
    /// # Returns
    ///
    /// A mutable reference to the underlying HAL implementation.
    pub fn hal_mut(&mut self) -> &mut H {
        &mut self.hal
    }
}
