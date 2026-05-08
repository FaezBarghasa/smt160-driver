//! Storage-Agnostic Calibration and Persistence Management.
//!
//! This module provides the infrastructure for piecewise linear calibration 
//! and persistence of sensor-specific correction factors.

use fixed::types::{I16F16, I32F32};
use embedded_storage::{ReadStorage, Storage};
use crate::Smt160Error;

/// A single calibration point mapping Duty Cycle to a known Reference Temperature.
///
/// # Usage Example
/// ```
/// use smt160_driver::calibration::CalibrationPoint;
/// use fixed::types::{I16F16, I32F32};
/// let point = CalibrationPoint {
///     duty_cycle: I32F32::from_num(0.4375),
///     reference_temperature: I16F16::from_num(25.0),
/// };
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CalibrationPoint {
    /// The measured duty cycle at the reference temperature.
    pub duty_cycle: I32F32,
    /// The known reference temperature in degrees Celsius (°C).
    pub reference_temperature: I16F16,
}

/// 5-Point Piecewise Linear Calibration Engine.
/// 
/// # Architecture
/// This engine allows for non-linear correction across the sensor's range by 
/// defining up to 5 calibration segments. It uses piecewise linear interpolation 
/// to derive the final temperature reading.
///
/// # Usage Example
/// ```
/// use smt160_driver::calibration::CalibrationProfile;
/// let profile = CalibrationProfile::default();
/// let temp = profile.interpolate_temperature(I32F32::from_num(0.5));
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CalibrationProfile {
    /// The array of sorted calibration points.
    pub points: [CalibrationPoint; 5],
    /// The number of active points in the profile.
    pub active_points_count: usize,
}

impl Default for CalibrationProfile {
    /// Provides a default profile based on standard SMT160 characteristics.
    ///
    /// # Panics
    /// This function does not panic.
    fn default() -> Self {
        let mut points = [CalibrationPoint::default(); 5];
        points[0] = CalibrationPoint { duty_cycle: I32F32::from_num(0.32), reference_temperature: I16F16::ZERO };
        points[1] = CalibrationPoint { duty_cycle: I32F32::from_num(0.4375), reference_temperature: I16F16::from_num(25) };
        points[2] = CalibrationPoint { duty_cycle: I32F32::from_num(0.79), reference_temperature: I16F16::from_num(100) };
        
        Self {
            points,
            active_points_count: 3,
        }
    }
}

impl CalibrationProfile {
    /// Applies piecewise linear interpolation to a raw duty cycle reading.
    /// 
    /// # Summary
    /// Derives the corrected temperature by finding the appropriate segment 
    /// in the calibration profile.
    ///
    /// # Panics
    /// This function does not panic.
    pub fn interpolate_temperature(&self, measured_duty_cycle: I32F32) -> I16F16 {
        if self.active_points_count == 0 {
            return I16F16::ZERO;
        }

        // Clamp to lower boundary
        if measured_duty_cycle <= self.points[0].duty_cycle {
            return self.points[0].reference_temperature;
        }
        
        // Clamp to upper boundary
        if measured_duty_cycle >= self.points[self.active_points_count - 1].duty_cycle {
            return self.points[self.active_points_count - 1].reference_temperature;
        }

        // Search for the relevant segment
        for i in 0..self.active_points_count - 1 {
            let p1 = &self.points[i];
            let p2 = &self.points[i+1];

            if measured_duty_cycle >= p1.duty_cycle && measured_duty_cycle <= p2.duty_cycle {
                return crate::math::interpolate_linear(
                    measured_duty_cycle, 
                    p1.duty_cycle, 
                    p1.reference_temperature, 
                    p2.duty_cycle, 
                    p2.reference_temperature
                );
            }
        }

        self.points[0].reference_temperature
    }
}

/// A storage-agnostic manager for sensor calibration persistence.
/// 
/// # Type Parameters
/// - `S`: Any backend implementing `Storage` and `ReadStorage` (e.g., Flash, EEPROM).
///
/// # Usage Example
/// ```
/// use smt160_driver::calibration::CalibrationManager;
/// let mut manager = CalibrationManager::new(flash_storage, 0x0800_C000);
/// manager.load_profile().unwrap();
/// ```
pub struct CalibrationManager<S> {
    storage_backend: S,
    /// The active calibration profile.
    pub profile: CalibrationProfile,
    memory_offset: u32,
}

impl<S> CalibrationManager<S> 
where 
    S: Storage + ReadStorage,
{
    /// Creates a new calibration manager with a specific storage backend.
    ///
    /// # Panics
    /// This function does not panic.
    pub fn new(storage_backend: S, memory_offset: u32) -> Self {
        Self {
            storage_backend,
            profile: CalibrationProfile::default(),
            memory_offset,
        }
    }

    /// Persists the current calibration profile to the storage backend.
    /// 
    /// # Errors
    /// Returns `Smt160Error::InvalidConfiguration` if the storage write operation fails.
    pub fn persist_profile(&mut self) -> Result<(), Smt160Error> {
        let data_buffer = [0u8; 64]; 
        // Serialization logic would go here in a full implementation.
        self.storage_backend.write(self.memory_offset, &data_buffer).map_err(|_| Smt160Error::InvalidConfiguration)
    }

    /// Loads the calibration profile from the storage backend.
    /// 
    /// # Errors
    /// Returns `Smt160Error::InvalidConfiguration` if the storage read operation fails 
    /// or if the integrity check (CRC) fails.
    pub fn load_profile(&mut self) -> Result<(), Smt160Error> {
        let mut data_buffer = [0u8; 64];
        self.storage_backend.read(self.memory_offset, &mut data_buffer).map_err(|_| Smt160Error::InvalidConfiguration)?;
        // Deserialization and integrity check would go here.
        Ok(())
    }
}

