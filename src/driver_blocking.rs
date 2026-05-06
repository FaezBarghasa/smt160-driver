//! High-Performance Polling-Based Driver for SMT160.

use crate::decoder::Smt160Decoder;
use crate::{Reading, Smt160Error};
use embedded_hal::digital::InputPin;

/// A synchronous, polling-based driver for high-precision temperature acquisition.
/// 
/// # Hazards
/// - **CPU Blocking**: This driver is entirely blocking and will consume 100% 
///   CPU time during the measurement cycle.
/// - **Critical Section Hazards**: The precision variant disables interrupts, 
///   which may cause missed deadlines in other system tasks.
pub struct Smt160BlockingDriver<P, T>
where
    P: InputPin,
    T: Fn() -> u64,
{
    input_pin: P,
    get_system_time_ticks: T,
    decoder: Smt160Decoder,
}

impl<P, T> Smt160BlockingDriver<P, T>
where
    P: InputPin,
    T: Fn() -> u64,
{
    /// Initializes a new polling-based driver.
    pub fn new(input_pin: P, get_system_time_ticks: T, decoder: Smt160Decoder) -> Self {
        Self {
            input_pin,
            get_system_time_ticks,
            decoder,
        }
    }

    /// Reads the temperature using a standard polling loop with a timeout guard.
    /// 
    /// # Summary
    /// Continuously polls the GPIO pin and processes edges until a full cycle is 
    /// completed or the timeout is reached.
    /// 
    /// # Errors
    /// Returns `Smt160Error::Timeout` if the measurement exceeds the specified duration.
    pub fn read_temperature_with_timeout(&mut self, timeout_ticks: u64) -> Result<Reading, Smt160Error> {
        let start_time = (self.get_system_time_ticks)();
        let mut last_captured_state = self.input_pin.is_high().map_err(|_| Smt160Error::Timeout)?;

        loop {
            let current_time = (self.get_system_time_ticks)();
            if current_time.wrapping_sub(start_time) > timeout_ticks {
                return Err(Smt160Error::Timeout);
            }

            let current_state = self.input_pin.is_high().map_err(|_| Smt160Error::Timeout)?;
            if current_state != last_captured_state {
                last_captured_state = current_state;
                
                use crate::config::{Smt160Config, StaticConfiguration};
                let (duty_cycle_offset, inverse_step_constant) = StaticConfiguration.get_offsets();
                
                if let Some(reading) = self.decoder.push_edge_timestamp(
                    current_state, 
                    current_time, 
                    duty_cycle_offset, 
                    inverse_step_constant
                )? {
                    return Ok(reading);
                }
            }
        }
    }

    /// Performs a high-precision measurement within a critical section.
    /// 
    /// # Summary
    /// Disables interrupts to minimize capture jitter, ensuring maximum 
    /// accuracy for single-point calibrations.
    /// 
    /// # Hazards
    /// Disables all system interrupts for approximately 25ms-50ms.
    pub fn read_temperature_high_precision(&mut self) -> Result<Reading, Smt160Error> {
        critical_section::with(|_| {
            self.decoder.reset_state();
            let mut transition_count = 0;
            let mut last_captured_state = self.input_pin.is_high().map_err(|_| Smt160Error::Timeout)?;
            
            // We need 3 transitions to capture one full cycle (Rise1 -> Fall -> Rise2)
            while transition_count < 100 { 
                let current_state = self.input_pin.is_high().map_err(|_| Smt160Error::Timeout)?;
                if current_state != last_captured_state {
                    let current_time = (self.get_system_time_ticks)();
                    last_captured_state = current_state;
                    transition_count += 1;
                    
                    use crate::config::{Smt160Config, StaticConfiguration};
                    let (duty_cycle_offset, inverse_step_constant) = StaticConfiguration.get_offsets();
                    
                    if let Some(reading) = self.decoder.push_edge_timestamp(
                        current_state, 
                        current_time, 
                        duty_cycle_offset, 
                        inverse_step_constant
                    )? {
                        return Ok(reading);
                    }
                }
            }
            
            Err(Smt160Error::SequenceViolation)
        })
    }
}
