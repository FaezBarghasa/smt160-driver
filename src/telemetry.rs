use bitflags::bitflags;

bitflags! {
    /// Industrial telemetry status for the SMT160 driver.
    /// 
    /// These flags allow the application layer to monitor the electrical 
    /// health of the sensor connection and the signal integrity.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct Smt160Status: u8 {
        /// Signal jitter exceeds 0.5% threshold. Indicates EMI or loose wiring.
        const JITTER_DETECTED = 1 << 0;
        
        /// Measurement is outside physical bounds (-45°C to 130°C).
        const OUT_OF_BOUNDS  = 1 << 1;
        
        /// Sensor pulse not detected for > 5ms. Indicates disconnection or hardware freeze.
        const SENSOR_TIMEOUT  = 1 << 2;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Smt160Status {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Smt160Status({:b})", self.bits());
    }
}

