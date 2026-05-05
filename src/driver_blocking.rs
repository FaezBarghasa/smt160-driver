use crate::decoder::Smt160Decoder;
use crate::Smt160Error;
use embedded_hal::digital::InputPin;
use fixed::types::I16F16;

/// A blocking driver for the SMT160 temperature sensor.
///
/// This driver explicitly loops (polls) while checking the pin state.
/// It uses a timeout loop counter (if no external timer is provided)
/// or relies on a user-provided clock closure.
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
    /// Creates a new `Smt160Blocking` driver.
    ///
    /// # Arguments
    /// * `pin` - The input pin connected to the sensor output.
    /// * `get_time_us` - A closure that returns the current time in microseconds.
    pub fn new(pin: P, get_time_us: T) -> Self {
        Self {
            pin,
            get_time_us,
            decoder: Smt160Decoder::new(),
        }
    }

    /// Reads the temperature using a busy-wait polling loop.
    ///
    /// This method will poll the pin and detect edges, feeding timestamps
    /// into the internal decoder until a stable temperature reading is produced.
    ///
    /// # Arguments
    /// * `timeout_us` - The maximum time (in microseconds) to wait for a successful reading.
    pub fn read_temperature(&mut self, timeout_us: u32) -> Result<I16F16, Smt160Error> {
        let start_time = (self.get_time_us)();
        let mut last_state = self.pin.is_high().unwrap_or(false);

        loop {
            let current_time = (self.get_time_us)();
            if current_time.saturating_sub(start_time) > timeout_us as u64 {
                return Err(Smt160Error::Timeout);
            }

            let current_state = self.pin.is_high().unwrap_or(false);
            if current_state != last_state {
                // Edge detected
                last_state = current_state;
                if let Some(temp) = self.decoder.push_edge(current_state, current_time)? {
                    return Ok(temp);
                }
            }
        }
    }
}
