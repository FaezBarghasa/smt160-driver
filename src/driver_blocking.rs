//! High-Precision Blocking Driver for SMT160.

use crate::decoder::Smt160Decoder;
use crate::Smt160Error;
use embedded_hal::digital::InputPin;
use fixed::types::I16F16;

/// A high-performance, polling-based driver for the SMT160.
pub struct Smt160Blocking<P, T>
where
    P: InputPin,
    T: Fn() -> u64,
{
    pin: P,
    get_time: T,
    decoder: Smt160Decoder,
}

impl<P, T> Smt160Blocking<P, T>
where
    P: InputPin,
    T: Fn() -> u64,
{
    /// Creates a new blocking driver.
    /// `get_time` should return timestamps in the same units as the decoder's clock.
    pub fn new(pin: P, get_time: T, decoder: Smt160Decoder) -> Self {
        Self {
            pin,
            get_time,
            decoder,
        }
    }

    /// Reads the temperature with a standard polling loop and timeout.
    /// `timeout` is in the same units as `get_time`.
    pub fn read_temperature(&mut self, timeout: u64) -> Result<I16F16, Smt160Error> {
        let start = (self.get_time)();
        let mut last_state = self.pin.is_high().map_err(|_| Smt160Error::Timeout)?;

        loop {
            let now = (self.get_time)();
            if now.wrapping_sub(start) > timeout {
                return Err(Smt160Error::Timeout);
            }

            let current_state = self.pin.is_high().map_err(|_| Smt160Error::Timeout)?;
            if current_state != last_state {
                last_state = current_state;
                if let Some(temp) = self.decoder.push_edge(current_state, now)? {
                    return Ok(temp);
                }
            }
        }
    }

    /// High-precision reading using a tight loop within a critical section.
    /// 
    /// This method minimizes jitter by:
    /// 1. Entering a `critical_section` to disable interrupts.
    /// 2. Performing a tight while-loop polling the GPIO pin.
    /// 3. Capturing timestamps immediately upon edge detection.
    /// 
    /// **WARNING**: This method blocks all other interrupts and tasks for at least
    /// one full sensor cycle (approx 0.25ms to 1ms). Use with caution in 
    /// real-time systems.
    pub fn read_temperature_precision(&mut self) -> Result<I16F16, Smt160Error> {
        critical_section::with(|_| {
            self.decoder.reset();
            let mut transitions = 0;
            let mut last_state = self.pin.is_high().map_err(|_| Smt160Error::Timeout)?;
            
            // We need 3 transitions to get one full cycle (Rise1, Fall, Rise2)
            while transitions < 3 {
                let current_state = self.pin.is_high().map_err(|_| Smt160Error::Timeout)?;
                if current_state != last_state {
                    let now = (self.get_time)();
                    last_state = current_state;
                    transitions += 1;
                    
                    if let Some(temp) = self.decoder.push_edge(current_state, now)? {
                        return Ok(temp);
                    }
                }
            }
            
            Err(Smt160Error::SequenceViolation)
        })
    }
}
