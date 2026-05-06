//! High-Accuracy Telemetry and I2C integration.

use core::sync::atomic::{AtomicU32, Ordering};
use fixed::types::I16F16;

/// Thread-safe telemetry container for sharing temperature data across tasks.
/// 
/// # Hazards
/// - **Stale Data**: If the sensor task stops updating, `get_latest` will return the last 
///   valid temperature indefinitely. Check sensor status separately if possible.
/// 
/// # Performance
/// - **Lock-Free**: Uses `AtomicU32` for non-blocking read/write access.
pub struct Smt160Telemetry {
    temp_bits: AtomicU32,
}

impl Smt160Telemetry {
    /// Creates a new telemetry container.
    pub const fn new() -> Self {
        Self {
            temp_bits: AtomicU32::new(0),
        }
    }

    /// Updates the stored temperature reading.
    pub fn update(&self, temp: I16F16) {
        self.temp_bits.store(temp.to_bits() as u32, Ordering::Relaxed);
    }

    /// Retrieves the latest temperature reading.
    pub fn get_latest(&self) -> I16F16 {
        I16F16::from_bits(self.temp_bits.load(Ordering::Relaxed) as i32)
    }

    /// Returns the latest temperature as a 4-byte LE array.
    pub fn get_latest_bytes(&self) -> [u8; 4] {
        self.temp_bits.load(Ordering::Relaxed).to_le_bytes()
    }
}

/// A non-blocking I2C telemetry task logic.
pub struct Smt160I2cTask<'a> {
    telemetry: &'a Smt160Telemetry,
}

impl<'a> Smt160I2cTask<'a> {
    pub fn new(telemetry: &'a Smt160Telemetry) -> Self {
        Self { telemetry }
    }

    /// Yields the latest temperature bytes. 
    /// 
    /// # Performance
    /// - **Deterministic**: Returns immediately without waiting for I2C bus locks.
    pub async fn handle_read_request(&self) -> [u8; 4] {
        self.telemetry.get_latest_bytes()
    }
}
