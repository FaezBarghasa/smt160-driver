use bitflags::bitflags;

bitflags! {
    /// Industrial telemetry status for the SMT160 driver.
    /// 
    /// These flags allow the application layer to monitor the electrical 
    /// health of the sensor connection and the signal integrity.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
        // Use a compact representation (hex bits) to save bandwidth
        defmt::write!(f, "S{=u8:02X}", self.bits());
    }
}

/// Diagnostic metrics for monitoring sensor health.
///
/// Uses Welford's online algorithm to calculate mean and standard deviation 
/// of raw timer ticks with O(1) space and time.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Diagnostics {
    pub mean_ticks: f32,
    pub m2_ticks: f32,
    pub count: u32,
    pub min_ticks: u32,
    pub max_ticks: u32,
    pub histogram: JitterHistogram,
}

impl Diagnostics {
    pub fn new() -> Self {
        Self { 
            mean_ticks: 0.0, 
            m2_ticks: 0.0, 
            count: 0,
            min_ticks: u32::MAX,
            max_ticks: 0,
            histogram: JitterHistogram::new(),
        }
    }

    /// Updates metrics with a new period measurement.
    pub fn update(&mut self, ticks: u32) {
        if ticks < self.min_ticks { self.min_ticks = ticks; }
        if ticks > self.max_ticks { self.max_ticks = ticks; }

        self.count = self.count.saturating_add(1);
        let x = ticks as f32;
        let delta = x - self.mean_ticks;
        self.mean_ticks += delta / self.count as f32;
        let delta2 = x - self.mean_ticks;
        self.m2_ticks += delta * delta2;

        self.histogram.update(ticks, self.mean_ticks);
    }

    #[cfg(feature = "std")]
    pub fn display_dashboard(&self) {
        println!("SMT160 INDUSTRIAL STABILITY DASHBOARD");
        println!("-------------------------------------");
        println!("Samples: {}", self.count);
        println!("Mean Period: {:.2} ticks", self.mean_ticks);
        println!("StdDev:      {:.2} ticks", self.std_dev());
        println!("Jitter P2P:  {} ticks", self.jitter_p2p());
        println!("\nJITTER DISTRIBUTION (HISTOGRAM):");
        let labels = ["<-20", "-20", "-10", "-5", "~0", "+5", "+10", "+20", ">20"];
        for (i, count) in self.histogram.counts.iter().enumerate() {
            let bar = "*".repeat((*count as f32 / self.count as f32 * 50.0) as usize);
            println!("{:>4}: {:>6} | {}", labels[i], count, bar);
        }
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

    /// Returns the peak-to-peak jitter in ticks.
    pub fn jitter_p2p(&self) -> u32 {
        if self.count == 0 { 0 } else { self.max_ticks - self.min_ticks }
    }
}

/// A compact histogram tracking period jitter distribution.
/// 
/// Buckets represent deviation from the mean in clock ticks.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct JitterHistogram {
    /// Buckets: < -20, -20..-11, -10..-6, -5..-2, -1..1, 2..5, 6..10, 11..20, > 20
    pub counts: [u32; 9],
}

impl JitterHistogram {
    pub const fn new() -> Self {
        Self { counts: [0; 9] }
    }

    pub fn update(&mut self, ticks: u32, mean: f32) {
        let diff = (ticks as f32 - mean) as i32;
        let bucket = if diff < -20 { 0 }
        else if diff < -10 { 1 }
        else if diff < -5 { 2 }
        else if diff < -1 { 3 }
        else if diff <= 1 { 4 }
        else if diff <= 5 { 5 }
        else if diff <= 10 { 6 }
        else if diff <= 20 { 7 }
        else { 8 };

        self.counts[bucket as usize] = self.counts[bucket as usize].saturating_add(1);
    }
}

