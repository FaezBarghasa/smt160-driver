use crate::decoder::Smt160Decoder;
use crate::{Reading, Smt160Error};
use embedded_hal::digital::InputPin;

/// A high-performance, polling-based driver for the SMT160.
/// 
/// # Hazards
/// - **Interrupt Latency**: In standard `read_temperature`, interrupt latency may cause jitter in 
///   timestamp capture, leading to noise in temperature readings.
/// - **CPU Blocking**: This driver is entirely blocking. It will consume 100% CPU while waiting for edges.
/// 
/// # Performance
/// - **Tight Loop**: The precision variant uses a tight polling loop to minimize capture jitter.
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
    pub fn read_temperature(&mut self, timeout: u64) -> Result<Reading, Smt160Error> {
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
                if let Some(reading) = self.decoder.push_edge(current_state, now)? {
                    return Ok(reading);
                }
            }
        }
    }

    /// High-precision reading using a tight loop within a critical section.
    /// 
    /// # Hazards
    /// - **Disables Interrupts**: This method disables interrupts for at least 1 sensor cycle (~25ms).
    ///   This may cause missed deadlines in high-speed control loops or UART parity errors.
    /// - **Blocks Execution**: This is a synchronous, blocking call. Do not use in real-time tasks 
    ///   with sub-10ms deadlines.
    pub fn read_temperature_precision(&mut self) -> Result<Reading, Smt160Error> {
        critical_section::with(|_| {
            self.decoder.reset();
            let mut transitions = 0;
            let mut last_state = self.pin.is_high().map_err(|_| Smt160Error::Timeout)?;
            
            // We need 3 transitions to get one full cycle (Rise1, Fall, Rise2)
            while transitions < 100 { // Allow some headroom
                let current_state = self.pin.is_high().map_err(|_| Smt160Error::Timeout)?;
                if current_state != last_state {
                    let now = (self.get_time)();
                    last_state = current_state;
                    transitions += 1;
                    
                    if let Some(reading) = self.decoder.push_edge(current_state, now)? {
                        return Ok(reading);
                    }
                }
            }
            
            Err(Smt160Error::SequenceViolation)
        })
    }
}
