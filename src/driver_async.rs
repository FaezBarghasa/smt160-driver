//! Async high-level driver for SMT160.

use crate::decoder::Smt160Decoder;
use crate::Smt160Error;
use embedded_hal::digital::InputPin;
use embedded_hal_async::digital::Wait;
use fixed::types::I16F16;

/// Async Wrapper for SMT160 utilizing native async traits.
pub struct Smt160Async<P, T> 
where 
    P: Wait + InputPin,
    T: Fn() -> u64,
{
    pin: P,
    get_time: T,
    decoder: Smt160Decoder,
}

impl<P, T> Smt160Async<P, T>
where
    P: Wait + InputPin,
    T: Fn() -> u64,
{
    /// Creates a new async driver.
    pub fn new(pin: P, get_time: T, decoder: Smt160Decoder) -> Self {
        Self {
            pin,
            get_time,
            decoder,
        }
    }

    /// Reads the temperature using 16-sample filtering.
    /// Returns the average temperature after 16 samples are collected.
    pub async fn read_temperature(&mut self) -> Result<I16F16, Smt160Error> {
        loop {
            // Await pin change
            self.pin.wait_for_any_edge().await.map_err(|_| Smt160Error::Timeout)?;

            // Capture timestamp immediately
            let now = (self.get_time)();

            // Determine edge
            let is_rising = self.pin.is_high().map_err(|_| Smt160Error::Timeout)?;

            // Process edge
            match self.decoder.push_edge(is_rising, now) {
                Ok(Some(temp)) => return Ok(temp),
                Ok(None) => continue,
                Err(e) => {
                    self.decoder.reset();
                    return Err(e);
                }
            }
        }
    }
}