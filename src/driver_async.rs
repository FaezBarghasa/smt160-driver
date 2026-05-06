//! Standard Asynchronous Wrapper for the SMT160 Sensor.

use crate::decoder::Smt160Decoder;
use crate::{Reading, Smt160Error};
use embedded_hal::digital::InputPin;
use embedded_hal_async::digital::Wait;

/// An asynchronous, non-blocking driver wrapper for GPIO-based pulse capture.
/// 
/// # Hazards
/// - **Context Switching Latency**: The precision of this driver depends on the latency 
///   of the async executor and hardware interrupt handling.
/// 
/// # Performance
/// - **Non-Blocking**: Yields back to the executor while waiting for edges, making 
///   it ideal for multitasking environments.
pub struct Smt160AsyncDriver<P, T> 
where 
    P: Wait + InputPin,
    T: Fn() -> u64,
{
    input_pin: P,
    get_system_time_ticks: T,
    decoder: Smt160Decoder,
}

impl<P, T> Smt160AsyncDriver<P, T>
where
    P: Wait + InputPin,
    T: Fn() -> u64,
{
    /// Initializes a new asynchronous driver.
    /// 
    /// # Summary
    /// `get_system_time_ticks` should provide monotonic timestamps in the same 
    /// resolution as specified in the `Smt160Decoder`.
    pub fn new(input_pin: P, get_system_time_ticks: T, decoder: Smt160Decoder) -> Self {
        Self {
            input_pin,
            get_system_time_ticks,
            decoder,
        }
    }

    /// Asynchronously captures and filters a full PWM cycle.
    /// 
    /// # Errors
    /// Returns `Smt160Error::Timeout` if the pin fails to transition or if 
    /// the signal violates physical boundaries.
    pub async fn read_temperature_celsius(&mut self) -> Result<Reading, Smt160Error> {
        loop {
            // Wait for any signal transition
            self.input_pin.wait_for_any_edge().await.map_err(|_| Smt160Error::Timeout)?;

            // Capture timestamp immediately to minimize jitter
            let current_timestamp = (self.get_system_time_ticks)();

            // Determine if the transition was a Rising or Falling edge
            let is_rising_edge = self.input_pin.is_high().map_err(|_| Smt160Error::Timeout)?;

            // Process the edge using standard manufacturer constants
            use crate::config::{Smt160Config, StaticConfiguration};
            let (duty_cycle_offset, inverse_step_constant) = StaticConfiguration.get_offsets();
            
            match self.decoder.push_edge_timestamp(
                is_rising_edge, 
                current_timestamp, 
                duty_cycle_offset, 
                inverse_step_constant
            ) {
                Ok(Some(reading)) => return Ok(reading),
                Ok(None) => continue, // Cycle incomplete, wait for next edge
                Err(error) => {
                    self.decoder.reset_state();
                    return Err(error);
                }
            }
        }
    }
}