#![no_std]

//! # SMT160 High-Precision Temperature Sensor Driver
//!
//! A professional, industrial-grade `no_std` driver for the SMT160 temperature sensor.
//! This driver implements a "Self-Documenting Clean Architecture" to ensure safety, 
//! clarity, and deterministic performance in embedded systems.

pub mod error;
pub mod types;
pub mod math;
pub mod conversion;
pub mod config;
pub mod decoder;
pub mod calibration;
pub mod platform;
pub mod telemetry;
pub mod driver_async;
pub mod driver_blocking;
pub mod i2c_telemetry;

#[cfg(feature = "stm32f1")]
pub mod stm32f1;

pub use error::Smt160Error;
pub use types::{Reading, Smt160Status, IndustrialHealth, ProcessingStats};

/// High-level generic SMT160 driver instance.
/// 
/// # Architecture
/// This driver uses a decoupled architecture where:
/// - **Configuration (`C`)**: Provides sensor calibration constants.
/// - **Capture Hardware (`CAP`)**: Abstracts the physical timer/capture peripheral.
///
/// # Usage Example
/// ```
/// use smt160_driver::Smt160Driver;
/// use smt160_driver::config::StaticConfiguration;
///
/// // Create a driver with static configuration and mock capture
/// let mut sensor = Smt160Driver::new(StaticConfiguration, mock_capture, 72);
/// ```
pub struct Smt160Driver<C, CAP> {
    /// The configuration provider for this driver.
    pub configuration: C,
    /// The hardware capture interface.
    pub capture_device: CAP,
    /// The internal decoding state machine.
    pub decoder: decoder::Smt160Decoder,
    /// Real-time industrial health metrics and diagnostics.
    pub health_monitor: IndustrialHealth,
}

impl<C, CAP> Smt160Driver<C, CAP> 
where 
    C: config::Smt160Config,
    CAP: platform::CaptureDevice<Error = Smt160Error>,
{
    /// Creates a new industrial SMT160 driver instance.
    /// 
    /// # Summary
    /// Initializes the driver with a specific configuration, hardware capture device, 
    /// and the expected timer clock frequency in Megahertz.
    /// 
    /// # Usage Example
    /// ```
    /// use smt160_driver::Smt160Driver;
    /// let driver = Smt160Driver::new(config, capture, 72);
    /// ```
    ///
    /// # Panics
    /// This function does not panic.
    pub fn new(configuration: C, capture_device: CAP, timer_clock_megahertz: u32) -> Self {
        Self {
            configuration,
            capture_device,
            decoder: decoder::Smt160Decoder::new_standalone(timer_clock_megahertz),
            health_monitor: IndustrialHealth::default(),
        }
    }

    /// Retrieves the latest industrial diagnostic health metrics.
    pub fn get_diagnostic_health(&self) -> IndustrialHealth {
        self.health_monitor
    }

    /// Performs a high-precision asynchronous temperature reading.
    /// 
    /// # Summary
    /// Waits for new capture data from the hardware and processes it through the 
    /// internal decoding engine.
    ///
    /// # Errors
    /// Returns `Smt160Error` if the signal is lost, frequency is out of range, 
    /// or if the duty cycle violates physical sensor boundaries.
    pub async fn read_temperature_celsius(&mut self) -> Result<Reading, Smt160Error> {
        self.capture_device.wait_for_new_data().await?;
        let (period_ticks, active_ticks) = self.capture_device.get_capture_data();
        let (duty_cycle_offset, inverse_step_constant) = self.configuration.get_offsets();
        
        self.decoder.process_raw_ticks(
            period_ticks, 
            active_ticks, 
            duty_cycle_offset, 
            inverse_step_constant
        )
    }
}

