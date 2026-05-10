use fixed::types::I32F32;
use embedded_storage::ReadStorage;
use crate::error::Smt160Error;

/// Trait for sensor calibration logic.
pub trait Calibration {
    /// Applies calibration to the raw temperature reading.
    fn calibrate(&self, temp: I32F32) -> I32F32;
}

/// A simple linear calibration (Slope and Offset).
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LinearCalibration {
    pub slope: I32F32,
    pub offset: I32F32,
}

impl Default for LinearCalibration {
    fn default() -> Self {
        Self {
            slope: I32F32::from_num(1.0),
            offset: I32F32::from_num(0.0),
        }
    }
}

impl Calibration for LinearCalibration {
    fn calibrate(&self, temp: I32F32) -> I32F32 {
        temp * self.slope + self.offset
    }
}

/// A piecewise linear calibration using up to 5 points.
/// 
/// This allows for correcting non-linear sensor responses across the 
/// entire temperature range.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PiecewiseLinearCalibration {
    pub points: [(I32F32, I32F32); 5],
    pub count: usize,
}

impl Default for PiecewiseLinearCalibration {
    fn default() -> Self {
        Self {
            points: [(I32F32::from_num(0), I32F32::from_num(0)); 5],
            count: 0,
        }
    }
}

impl Calibration for PiecewiseLinearCalibration {
    fn calibrate(&self, temp: I32F32) -> I32F32 {
        if self.count == 0 {
            return temp;
        }
        if self.count == 1 {
            return temp + self.points[0].1 - self.points[0].0;
        }

        // Clamp to extremes
        if temp <= self.points[0].0 {
            return self.points[0].1;
        }
        if temp >= self.points[self.count - 1].0 {
            return self.points[self.count - 1].1;
        }

        // Search for the interval
        for i in 0..self.count - 1 {
            let (x0, y0) = self.points[i];
            let (x1, y1) = self.points[i + 1];

            if temp >= x0 && temp <= x1 {
                let dx = x1 - x0;
                let dy = y1 - y0;
                return y0 + (temp - x0) * dy / dx;
            }
        }
        temp
    }
}

use embedded_storage::Storage;

/// Magic number to identify valid calibration data in Flash.
const CALIB_MAGIC: u32 = 0x534D5431; // "SMT1"
const CALIB_VERSION: u16 = 1;

/// Helper to load calibration from storage.
pub fn load_calibration<S: ReadStorage>(storage: &mut S, address: u32) -> Result<LinearCalibration, Smt160Error> {
    let mut magic_buf = [0u8; 4];
    storage.read(address, &mut magic_buf).map_err(|_| Smt160Error::InvalidBuffer)?;
    if u32::from_le_bytes(magic_buf) != CALIB_MAGIC {
        return Err(Smt160Error::InvalidSignal); // Using InvalidSignal as a placeholder for InvalidCalibration
    }

    let mut buf = [0u8; 8];
    storage.read(address + 8, &mut buf).map_err(|_| Smt160Error::InvalidBuffer)?;
    let slope = I32F32::from_bits(i64::from_le_bytes(buf));
    
    storage.read(address + 16, &mut buf).map_err(|_| Smt160Error::InvalidBuffer)?;
    let offset = I32F32::from_bits(i64::from_le_bytes(buf));
    
    Ok(LinearCalibration { slope, offset })
}

/// Helper to save calibration to storage.
pub fn save_calibration<S: Storage>(storage: &mut S, address: u32, cal: &LinearCalibration) -> Result<(), Smt160Error> {
    storage.write(address, &CALIB_MAGIC.to_le_bytes()).map_err(|_| Smt160Error::InvalidBuffer)?;
    storage.write(address + 4, &CALIB_VERSION.to_le_bytes()).map_err(|_| Smt160Error::InvalidBuffer)?;
    storage.write(address + 8, &cal.slope.to_bits().to_le_bytes()).map_err(|_| Smt160Error::InvalidBuffer)?;
    storage.write(address + 16, &cal.offset.to_bits().to_le_bytes()).map_err(|_| Smt160Error::InvalidBuffer)?;
    Ok(())
}
