use crate::decoder::Smt160Decoder;
use crate::Smt160Error;
use embedded_hal::digital::InputPin;
use fixed::types::I16F16;

/// A high-performance, polling-based driver for the SMT160 temperature sensor.
///
/// This driver is designed for environments where an async executor is not available
/// or where the sensor must be read in a dedicated, high-priority loop. It relies on
/// a user-provided timestamp source (e.g., a hardware timer) to calculate the duty cycle
/// and frequency of the SMT160 signal.
///
/// ### Characteristics
/// - **Zero-Allocation**: No heap usage, entirely stack-based.
/// - **Polling Architecture**: Busy-waits on pin transitions to ensure maximum edge accuracy.
/// - **Configurable Timebase**: Compatible with any hardware timer that can provide a microsecond-scale timestamp.
///
/// ### Example
/// ```rust
/// use smt160_driver::driver_blocking::Smt160Blocking;
/// 
/// // Assume 'pin' is an InputPin and 'get_time' is a closure returning microseconds.
/// let mut sensor = Smt160Blocking::new(pin, || timer.now());
/// 
/// match sensor.read_temperature(100_000) { // 100ms timeout
///     Ok(temp) => println!("Temperature: {}°C", temp),
///     Err(e) => println!("Error: {:?}", e),
/// }
/// ```
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
    /// Creates a new `Smt160Blocking` driver instance.
    ///
    /// # Arguments
    /// * `pin` - A digital input pin implementing `embedded_hal::digital::InputPin`.
    /// * `get_time_us` - A closure or function pointer returning a 64-bit microsecond timestamp.
    pub fn new(pin: P, get_time_us: T) -> Self {
        Self {
            pin,
            get_time_us,
            decoder: Smt160Decoder::new(),
        }
    }

    /// Reads the temperature from the sensor using a synchronous polling loop.
    ///
    /// This method will continuously poll the input pin until the internal decoder
    /// has gathered enough edges (typically 2-3 full cycles) to produce a stable reading.
    ///
    /// # Safety & Performance
    /// This is a **blocking** operation. It will monopolize the CPU core until a reading 
    /// is complete or the timeout is reached. For non-blocking applications, use `Smt160Async`.
    ///
    /// # Arguments
    /// * `timeout_us` - Maximum duration to wait for a valid reading before returning `Smt160Error::Timeout`.
    ///
    /// # Returns
    /// * `Ok(I16F16)` - The decoded temperature in degrees Celsius (fixed-point).
    /// * `Err(Smt160Error)` - Encountered an error (timeout, jitter, out of range, etc).
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
