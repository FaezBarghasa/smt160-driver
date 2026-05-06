//! Thread-Safe I2C Telemetry and Shared State Management.

use core::sync::atomic::{AtomicU32, Ordering};
use fixed::types::I16F16;

/// A thread-safe, lock-free container for sharing temperature data across task boundaries.
/// 
/// # Architecture
/// Utilizes atomic primitives (`AtomicU32`) to allow high-frequency updates from a 
/// sensor task while providing non-blocking access for I2C read requests.
pub struct Smt160SharedTelemetry {
    temperature_bits_atomic: AtomicU32,
}


impl Default for Smt160SharedTelemetry {
    fn default() -> Self {
        Self::new()
    }
}

impl Smt160SharedTelemetry {
    /// Creates a new shared telemetry container initialized to zero.
    pub const fn new() -> Self {
        Self {
            temperature_bits_atomic: AtomicU32::new(0),
        }
    }

    /// Atomically updates the stored temperature reading.
    pub fn update_temperature(&self, current_temperature: I16F16) {
        self.temperature_bits_atomic.store(current_temperature.to_bits() as u32, Ordering::Release);
    }

    /// Retrieves the latest temperature reading in fixed-point format.
    pub fn get_latest_reading(&self) -> I16F16 {
        I16F16::from_bits(self.temperature_bits_atomic.load(Ordering::Acquire) as i32)
    }

    /// Returns the latest temperature as a 4-byte little-endian byte array.
    /// 
    /// # Summary
    /// Optimized for direct transmission over I2C or SPI buses.
    pub fn get_latest_reading_bytes(&self) -> [u8; 4] {
        self.temperature_bits_atomic.load(Ordering::Acquire).to_le_bytes()
    }
}

/// Logic for handling asynchronous I2C telemetry requests.
pub struct Smt160I2cTelemetryTask<'a> {
    shared_telemetry: &'a Smt160SharedTelemetry,
}

impl<'a> Smt160I2cTelemetryTask<'a> {
    /// Creates a new I2C task logic handler.
    pub fn new(shared_telemetry: &'a Smt160SharedTelemetry) -> Self {
        Self { shared_telemetry }
    }

    /// Processes an incoming I2C read request and returns the latest temperature bytes.
    /// 
    /// # Performance
    /// - **Deterministic**: Returns immediately without yielding or blocking.
    pub async fn handle_i2c_read_request(&self) -> [u8; 4] {
        self.shared_telemetry.get_latest_reading_bytes()
    }
}
