//! Data Conversion and Bit Manipulation Utilities for SMT160.
//!
//! This module contains low-level utilities for serializing fixed-point data 
//! and unpacking hardware-specific capture formats.

use fixed::types::I16F16;

/// Converts a 32-bit little-endian byte array to a 16.16 fixed-point number.
/// 
/// # Summary
/// Useful for deserializing temperature readings from non-volatile storage or telemetry.
/// 
/// # Usage Example
/// ```
/// use smt160_driver::conversion::bytes_to_i16f16;
/// use fixed::types::I16F16;
/// let bytes = [0x00, 0x00, 0x01, 0x00]; // 1.0 in fixed-point bits
/// let result = bytes_to_i16f16(bytes);
/// assert_eq!(result, I16F16::from_num(1.0));
/// ```
pub fn bytes_to_i16f16(bytes: [u8; 4]) -> I16F16 {
    I16F16::from_bits(i32::from_le_bytes(bytes))
}

/// Converts a 16.16 fixed-point number to a 32-bit little-endian byte array.
/// 
/// # Summary
/// Prepares a temperature reading for transmission or storage.
/// 
/// # Usage Example
/// ```
/// use smt160_driver::conversion::i16f16_to_bytes;
/// use fixed::types::I16F16;
/// let value = I16F16::from_num(1.0);
/// let bytes = i16f16_to_bytes(value);
/// assert_eq!(bytes, [0x00, 0x00, 0x01, 0x00]);
/// ```
pub fn i16f16_to_bytes(value: I16F16) -> [u8; 4] {
    value.to_bits().to_le_bytes()
}

/// Unpacks a 32-bit DMA capture word into period and active ticks.
/// 
/// # Summary
/// Expected format: `(16-bit Period << 16 | 16-bit Active)`. This is a common 
/// hardware format for STM32 PWM input capture.
/// 
/// # Usage Example
/// ```
/// use smt160_driver::conversion::unpack_dma_capture;
/// let word = 0x03E8_01F4; // Period: 1000, Active: 500 (50% Duty Cycle)
/// let (period, active) = unpack_dma_capture(word);
/// assert_eq!(period, 1000);
/// assert_eq!(active, 500);
/// ```
pub fn unpack_dma_capture(word: u32) -> (u64, u64) {
    // STM32 Standard: CCR1 (Period) is at lower address, CCR2 (Active) at higher.
    // In a 32-bit word, CCR1 is the lower 16 bits.
    let period_ticks = (word & 0xFFFF) as u64;
    let active_ticks = (word >> 16) as u64;
    (period_ticks, active_ticks)
}

