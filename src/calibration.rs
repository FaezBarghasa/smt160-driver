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

/// Helper to load calibration from storage.
pub fn load_calibration<S: ReadStorage>(storage: &mut S, address: u32) -> Result<LinearCalibration, Smt160Error> {
    let mut buf = [0u8; 8];
    storage.read(address, &mut buf).map_err(|_| Smt160Error::InvalidBuffer)?;
    
    let slope_bits = i64::from_le_bytes(buf);
    let slope = I32F32::from_bits(slope_bits);
    
    let mut buf = [0u8; 8];
    storage.read(address + 8, &mut buf).map_err(|_| Smt160Error::InvalidBuffer)?;
    let offset_bits = i64::from_le_bytes(buf);
    let offset = I32F32::from_bits(offset_bits);
    
    Ok(LinearCalibration { slope, offset })
}
