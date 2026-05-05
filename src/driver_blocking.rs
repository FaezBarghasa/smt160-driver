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
    get_time_us: T,
    decoder: Smt160Decoder,
}

impl<P, T> Smt160Blocking<P, T>
where
    P: InputPin,
    T: Fn() -> u64,
{
    pub fn new(pin: P, get_time_us: T) -> Self {
        Self {
            pin,
            get_time_us,
            decoder: Smt160Decoder::new(),
        }
    }

    /// Reads the temperature with a standard polling loop and timeout.
    pub fn read_temperature(&mut self, timeout_us: u32) -> Result<I16F16, Smt160Error> {
        let start = (self.get_time_us)();
        let mut last_state = self.pin.is_high().map_err(|_| Smt160Error::Timeout)?;

        loop {
            let now = (self.get_time_us)();
            if now.wrapping_sub(start) > timeout_us as u64 {
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
    /// This minimizes jitter from interrupts during the measurement of 3 transitions.
    pub fn read_temperature_precision(&mut self) -> Result<I16F16, Smt160Error> {
        critical_section::with(|_| {
            self.decoder.reset();
            let mut transitions = 0;
            let mut last_state = self.pin.is_high().map_err(|_| Smt160Error::Timeout)?;
            
            // We need 3 transitions to get one full cycle (Rise1, Fall, Rise2)
            while transitions < 3 {
                let current_state = self.pin.is_high().map_err(|_| Smt160Error::Timeout)?;
                if current_state != last_state {
                    let now = (self.get_time_us)();
                    last_state = current_state;
                    transitions += 1;
                    
                    if let Some(temp) = self.decoder.push_edge(current_state, now)? {
                        return Ok(temp);
                    }
                }
            }
            
            // If we didn't get a result after 3 transitions, something is wrong with the decoder state
            Err(Smt160Error::SequenceViolation)
        })
    }
}
