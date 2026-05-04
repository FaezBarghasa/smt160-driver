//! Telemetry and I2C integration.
//!
//! This module provides tools for sharing temperature data between the sensor 
//! decoder task and external communication interfaces like I2C.

use core::sync::atomic::{AtomicU32, Ordering};
use fixed::types::I16F16;
use crate::Smt160Error;

/// Thread-safe telemetry container for sharing temperature data across tasks.
///
/// Uses an `AtomicU32` to store the fixed-point bits, allowing high-priority 
/// interrupt tasks to update the value without locking low-priority telemetry tasks.
pub struct Smt160Telemetry {
    temp_bits: AtomicU32,
}

impl Smt160Telemetry {
    /// Creates a new telemetry container initialized to 0°C.
    pub const fn new() -> Self {
        Self {
            temp_bits: AtomicU32::new(0),
        }
    }

    /// Updates the stored temperature reading.
    ///
    /// This should be called by the SMT160 driver task or interrupt handler.
    pub fn update(&self, temp: I16F16) {
        self.temp_bits.store(temp.to_bits() as u32, Ordering::Relaxed);
    }

    /// Retrieves the latest temperature reading as `I16F16`.
    pub fn get_latest(&self) -> I16F16 {
        I16F16::from_bits(self.temp_bits.load(Ordering::Relaxed) as i32)
    }

    /// Formats the latest temperature into a 4-byte little-endian array for I2C transmission.
    pub fn get_latest_bytes(&self) -> [u8; 4] {
        let bits = self.temp_bits.load(Ordering::Relaxed);
        bits.to_le_bytes()
    }
}

/// A non-blocking I2C telemetry responder logic.
///
/// This provides the processing layer for responding to I2C read requests.
/// Note: Standard async I2C slave traits are HAL-specific; this struct
/// prepares data for such implementations.
pub struct Smt160I2cResponder<'a, I2C> {
    telemetry: &'a Smt160Telemetry,
    _i2c: core::marker::PhantomData<I2C>,
}

impl<'a, I2C> Smt160I2cResponder<'a, I2C> 
where 
    I2C: embedded_hal_async::i2c::I2c
{
    /// Creates a new responder linked to a telemetry source.
    pub fn new(telemetry: &'a Smt160Telemetry) -> Self {
        Self {
            telemetry,
            _i2c: core::marker::PhantomData,
        }
    }

    /// Prepares telemetry bytes for an I2C read request.
    ///
    /// In a real system, this would be invoked by an I2C slave event handler.
    pub async fn process_telemetry_request(&self) -> Result<[u8; 4], Smt160Error> {
        Ok(self.telemetry.get_latest_bytes())
    }
}

