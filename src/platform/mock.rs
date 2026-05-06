//! Virtual Mock Implementation of SMT160 Capture for Testing.

use crate::platform::CaptureDevice;
use core::future::Future;

/// A virtual capture device that returns deterministic, pre-defined values.
/// 
/// # Usage Example
/// ```
/// let mut mock = VirtualCapture::new(1000, 438);
/// let (period, active) = mock.get_capture_data();
/// ```
pub struct VirtualCapture {
    /// The virtual period ticks to return.
    pub period_ticks: u64,
    /// The virtual active high-time ticks to return.
    pub active_ticks: u64,
    /// Whether the mock data is ready for "capture".
    pub is_data_ready: bool,
}

impl VirtualCapture {
    /// Creates a new virtual capture device with specific test data.
    pub fn new(period_ticks: u64, active_ticks: u64) -> Self {
        Self { 
            period_ticks, 
            active_ticks, 
            is_data_ready: true 
        }
    }
}

impl CaptureDevice for VirtualCapture {
    type Error = crate::Smt160Error;

    /// Returns the mocked period and active ticks.
    fn get_capture_data(&self) -> (u64, u64) {
        (self.period_ticks, self.active_ticks)
    }

    /// Asynchronously yields if data is ready, otherwise pends.
    async fn wait_for_new_data(&mut self) -> Result<(), Self::Error> {
        if self.is_data_ready {
            Ok(())
        } else {
            core::future::pending().await
        }
    }
}
