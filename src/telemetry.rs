use bitflags::bitflags;

bitflags! {
    /// Industrial diagnostic status flags for the SMT160 sensor.
    ///
    /// These flags are bit-packed to allow for efficient telemetry transmission 
    /// and real-time monitoring without heap allocation.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Smt160Status: u8 {
        /// Signal jitter exceeds the 0.5% safety threshold. 
        /// Indicates potential EMI or wiring issues.
        const JITTER_DETECTED = 0b0000_0001;
        
        /// The decoded temperature or duty cycle is outside physical bounds.
        const OUT_OF_BOUNDS  = 0b0000_0010;
        
        /// No signal pulses detected within the watchdog window.
        const SENSOR_TIMEOUT = 0b0000_0100;
    }
}
