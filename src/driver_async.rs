use crate::decoder::Smt160Decoder;
use crate::{Reading, Smt160Error};
use embedded_hal::digital::InputPin;
use embedded_hal_async::digital::Wait;

/// Async Wrapper for SMT160 utilizing native async traits.
/// 
/// # Hazards
/// - **Context Switching Latency**: The precision of this driver depends on the latency of the 
///   async executor and the underlying hardware interrupt handling.
/// 
/// # Performance
/// - **Non-Blocking**: This driver yields back to the executor while waiting for edges, making 
///   it ideal for concurrent applications.
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

    /// Reads the temperature using high-precision filtering.
    /// Returns the filtered temperature reading once a full cycle is processed.
    pub async fn read_temperature(&mut self) -> Result<Reading, Smt160Error> {
        loop {
            // Await pin change
            self.pin.wait_for_any_edge().await.map_err(|_| Smt160Error::Timeout)?;

            // Capture timestamp immediately
            let now = (self.get_time)();

            // Determine edge
            let is_rising = self.pin.is_high().map_err(|_| Smt160Error::Timeout)?;

            // Process edge
            match self.decoder.push_edge(is_rising, now) {
                Ok(Some(reading)) => return Ok(reading),
                Ok(None) => continue,
                Err(e) => {
                    self.decoder.reset();
                    return Err(e);
                }
            }
        }
    }
}