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

        /// Temperature change rate exceeds 10°C/s.
        const GRADIENT_ERROR = 1 << 3;

        /// Signal jitter exceeds 1.5% of mean period.
        const SIGNAL_NOISY = 1 << 4;

        /// Temperature change rate exceeds 15°C/s.
        const THERMAL_RUNAWAY = 1 << 5;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Smt160Status {
    fn format(&self, f: defmt::Formatter) {
        // Use a compact representation (hex bits) to save bandwidth
        defmt::write!(f, "S{=u8:02X}", self.bits());
    }
}

use fixed::types::I32F32;

use portable_atomic::{AtomicU32, AtomicU64};
use core::sync::atomic::Ordering;

/// Diagnostic metrics for monitoring sensor health.
///
/// Uses Welford's online algorithm with atomic updates to allow 
/// concurrent health monitoring without locking.
pub struct Diagnostics {
    pub mean_ticks: AtomicU64, // bits of I32F32
    pub m2_ticks: AtomicU64,   // bits of I32F32
    pub count: AtomicU32,
    pub min_ticks: AtomicU32,
    pub max_ticks: AtomicU32,
    pub histogram: JitterHistogram,
}

impl Diagnostics {
    pub fn new() -> Self {
        Self { 
            mean_ticks: AtomicU64::new(0), 
            m2_ticks: AtomicU64::new(0), 
            count: AtomicU32::new(0),
            min_ticks: AtomicU32::new(u32::MAX),
            max_ticks: AtomicU32::new(0),
            histogram: JitterHistogram::new(),
        }
    }

    /// Updates metrics with a new period measurement.
    pub fn update(&mut self, ticks: u32) {
        // Update Min/Max (not strictly atomic across whole struct but safe for individual fields)
        let mut current_min = self.min_ticks.load(Ordering::Relaxed);
        while ticks < current_min {
            match self.min_ticks.compare_exchange_weak(current_min, ticks, Ordering::Relaxed, Ordering::Relaxed) {
                Ok(_) => break,
                Err(new_min) => current_min = new_min,
            }
        }

        let mut current_max = self.max_ticks.load(Ordering::Relaxed);
        while ticks > current_max {
            match self.max_ticks.compare_exchange_weak(current_max, ticks, Ordering::Relaxed, Ordering::Relaxed) {
                Ok(_) => break,
                Err(new_max) => current_max = new_max,
            }
        }

        let count = self.count.fetch_add(1, Ordering::Relaxed) + 1;
        let x = I32F32::from_num(ticks);
        
        // Atomic Welford's Update (simplified to critical section for consistency of mean/m2)
        critical_section::with(|_| {
            let mean_bits = self.mean_ticks.load(Ordering::Relaxed);
            let m2_bits = self.m2_ticks.load(Ordering::Relaxed);
            
            let mut mean = I32F32::from_bits(mean_bits as i64);
            let mut m2 = I32F32::from_bits(m2_bits as i64);
            
            let delta = x - mean;
            mean += delta / I32F32::from_num(count);
            let delta2 = x - mean;
            m2 += delta * delta2;
            
            self.mean_ticks.store(mean.to_bits() as u64, Ordering::Relaxed);
            self.m2_ticks.store(m2.to_bits() as u64, Ordering::Relaxed);
        });

        self.histogram.update(ticks, I32F32::from_bits(self.mean_ticks.load(Ordering::Relaxed) as i64));
    }

    /// Returns the variance of captured ticks.
    pub fn variance(&self) -> I32F32 {
        let count = self.count.load(Ordering::Relaxed);
        if count < 2 {
            I32F32::from_num(0)
        } else {
            let m2_bits = self.m2_ticks.load(Ordering::Relaxed);
            let m2 = I32F32::from_bits(m2_bits as i64);
            m2 / I32F32::from_num(count - 1)
        }
    }

    /// Returns the standard deviation of captured ticks.
    pub fn std_dev(&self) -> I32F32 {
        let v = self.variance();
        if v > 0 {
            I32F32::from_num(libm::sqrt(v.to_num::<f64>()))
        } else {
            I32F32::from_num(0)
        }
    }

    /// Returns the RMS Jitter (Standard Deviation of the period).
    pub fn jitter_rms(&self) -> I32F32 {
        self.std_dev()
    }

    pub fn mean_period(&self) -> I32F32 {
        I32F32::from_bits(self.mean_ticks.load(Ordering::Relaxed) as i64)
    }

    pub fn jitter_p2p(&self) -> u32 {
        let max = self.max_ticks.load(Ordering::Relaxed);
        let min = self.min_ticks.load(Ordering::Relaxed);
        if max >= min { max - min } else { 0 }
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

    pub fn update(&mut self, ticks: u32, mean: I32F32) {
        let diff = (I32F32::from_num(ticks) - mean).to_num::<i32>();
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

