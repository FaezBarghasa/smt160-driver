//! Hardware Abstraction Layer (HAL) for the SMT160 Sensor.
//! 
//! This module defines the standardized traits required for hardware capture 
//! implementations. By implementing `CaptureDevice`, new microcontrollers can 
//! be supported without modifying the core decoding logic.

use core::future::Future;

/// Abstract interface for hardware PWM pulse capture.
/// 
/// # Summary
/// This trait defines the contract for hardware-specific drivers that capture 
/// the period and active time of the SMT160 PWM signal.
pub trait CaptureDevice {
    /// Errors specific to the hardware capture peripheral.
    type Error;

    /// Retrieves the raw pulse data from the most recent capture cycle.
    /// 
    /// # Returns
    /// A tuple containing `(Period_Ticks, Active_Ticks)`.
    fn get_capture_data(&self) -> (u64, u64);

    /// Asynchronous hook that suspends the task until new pulse data is available.
    /// 
    /// # Errors
    /// Returns the underlying hardware error if the capture peripheral fails.
    fn wait_for_new_data(&mut self) -> impl Future<Output = Result<(), Self::Error>>;
}
