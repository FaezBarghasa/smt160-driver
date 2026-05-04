//! Async high-level driver for SMT160.
//!
//! This module provides a convenient async interface for reading the SMT160 
//! sensor using `embedded-hal-async` traits.

use crate::decoder::Smt160Decoder;
use crate::Smt160Error;
use embedded_hal::digital::InputPin;
use embedded_hal_async::digital::Wait;
use fixed::types::I16F16;

/// Async Wrapper for SMT160 utilizing native async traits.
///
/// This driver wraps a GPIO pin and a timer function to provide 
/// high-level temperature readings.
pub struct Smt160Async<P, T> {
    pin: P,
    timer_fn: T,
    decoder: Smt160Decoder,
}

impl<P, T> Smt160Async<P, T>
where
    P: Wait + InputPin,
    T: Fn() -> u64,
{
    /// Creates a new async driver.
    ///
    /// # Arguments
    /// * `pin` - A GPIO pin implementing `Wait` and `InputPin`.
    /// * `timer_fn` - A closure or function that returns the current time in microseconds.
    pub fn new(pin: P, timer_fn: T) -> Self {
        Self {
            pin,
            timer_fn,
            decoder: Smt160Decoder::new(),
        }
    }

    /// Reads the temperature. 
    ///
    /// This method is **cancellation safe**. If the future is dropped before completion,
    /// the internal state is reset on the next call to ensure no partial pulse data
    /// pollutes the new reading.
    ///
    /// # Returns
    /// * `Ok(I16F16)` - The verified temperature reading.
    /// * `Err(Smt160Error)` - Encountered a hardware or signal error.
    pub async fn read_temperature(&mut self) -> Result<I16F16, Smt160Error> {
        // Reset state so dropping this future mid-pulse previously won't corrupt our readings.
        self.decoder.reset();

        loop {
            // Await any pin change (both rising and falling edges)
            self.pin.wait_for_any_edge().await.map_err(|_| Smt160Error::Timeout)?;

            // Critical Section: Capture the time *immediately* after waking up
            let timestamp_us = (self.timer_fn)();

            // Resolve edge direction
            let is_rising = self.pin.is_high().map_err(|_| Smt160Error::Timeout)?;

            // Pass the data to the passive engine
            match self.decoder.push_edge(is_rising, timestamp_us) {
                Ok(Some(temperature)) => return Ok(temperature),
                Ok(None) => continue, // Engine needs more pulses to stabilize; keep looping
                Err(e) => return Err(e),
            }
        }
    }
}