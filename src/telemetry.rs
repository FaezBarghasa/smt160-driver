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

/// Diagnostic metrics for monitoring sensor health.
///
/// Uses Welford's online algorithm to calculate mean and standard deviation 
/// of raw timer ticks with O(1) space and time.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Diagnostics {
    pub mean_ticks: f32,
    pub m2_ticks: f32,
    pub count: u32,
}

impl Diagnostics {
    pub fn new() -> Self {
        Self { mean_ticks: 0.0, m2_ticks: 0.0, count: 0 }
    }

    /// Updates metrics with a new period measurement.
    pub fn update(&mut self, ticks: u32) {
        self.count = self.count.saturating_add(1);
        let x = ticks as f32;
        let delta = x - self.mean_ticks;
        self.mean_ticks += delta / self.count as f32;
        let delta2 = x - self.mean_ticks;
        self.m2_ticks += delta * delta2;
    }

    /// Returns the standard deviation of captured ticks.
    /// High variance often indicates EMI or connector failure.
    pub fn std_dev(&self) -> f32 {
        if self.count < 2 {
            0.0
        } else {
            // Using libm for no_std sqrt
            libm::sqrtf(self.m2_ticks / (self.count - 1) as f32)
        }
    }
}

