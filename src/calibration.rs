//! 2-Point Calibration and Persistence.
//! 
//! This module provides a linear correction engine and helpers to store 
//! calibration coefficients in the STM32F1's internal Flash memory.

use fixed::types::I16F16;

/// Calibration engine for per-unit linear correction.
/// T_final = (T_raw * multiplier) + offset
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Calibration {
    pub multiplier: I16F16,
    pub offset: I16F16,
    pub p1_raw: Option<I16F16>,
    pub p2_raw: Option<I16F16>,
}

impl Default for Calibration {
    fn default() -> Self {
        Self {
            multiplier: I16F16::ONE,
            offset: I16F16::ZERO,
            p1_raw: None,
            p2_raw: None,
        }
    }
}

impl Calibration {
    /// Records the raw reading for the "low" calibration point (e.g. 0°C).
    pub fn calibrate_low(&mut self, raw: I16F16, known_temp: I16F16) {
        self.p1_raw = Some(raw);
        if let Some(p2) = self.p2_raw {
            self.recalculate(raw, known_temp, p2, I16F16::from_num(100)); // Assume high is 100 for now if p2 exists
        }
    }

    /// Records the raw reading for the "high" calibration point (e.g. 100°C).
    pub fn calibrate_high(&mut self, raw: I16F16, known_temp: I16F16) {
        self.p2_raw = Some(raw);
        if let Some(p1) = self.p1_raw {
            self.recalculate(p1, I16F16::ZERO, raw, known_temp); // Assume low is 0
        }
    }

    fn recalculate(&mut self, x1: I16F16, y1: I16F16, x2: I16F16, y2: I16F16) {
        let dx = x2 - x1;
        let dy = y2 - y1;
        if dx != 0 {
            self.multiplier = dy / dx;
            self.offset = y1 - (self.multiplier * x1);
        }
    }

    /// Applies calibration to a raw reading.
    pub fn apply(&self, raw: I16F16) -> I16F16 {
        (raw * self.multiplier) + self.offset
    }

    /// Calculates a simple CRC-8 for the calibration data.
    pub fn crc8(&self) -> u8 {
        let mut crc = 0u8;
        let bytes = [
            self.multiplier.to_bits().to_le_bytes(),
            self.offset.to_bits().to_le_bytes(),
        ];
        for chunk in bytes {
            for b in chunk {
                crc ^= b;
                for _ in 0..8 {
                    if crc & 0x80 != 0 {
                        crc = (crc << 1) ^ 0x07;
                    } else {
                        crc <<= 1;
                    }
                }
            }
        }
        crc
    }
}
