pub mod mock;
pub mod stm32f1;

#[cfg(feature = "stm32f1")]
pub mod stm32f1_managed;

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
