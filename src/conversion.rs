//! Data Conversion and Bit Manipulation Utilities for SMT160.

use fixed::types::I16F16;

/// Converts a 32-bit little-endian byte array to a 16.16 fixed-point number.
/// 
/// # Usage Example
/// ```
/// let bytes = [0x00, 0x00, 0x01, 0x00]; // 1.0 in bits
/// let result = smt160_driver::conversion::bytes_to_i16f16(bytes);
/// assert_eq!(result, I16F16::from_num(1.0));
/// ```
pub fn bytes_to_i16f16(bytes: [u8; 4]) -> I16F16 {
    I16F16::from_bits(i32::from_le_bytes(bytes))
}

/// Converts a 16.16 fixed-point number to a 32-bit little-endian byte array.
pub fn i16f16_to_bytes(value: I16F16) -> [u8; 4] {
    value.to_bits().to_le_bytes()
}

/// Unpacks a 32-bit DMA capture word into period and active ticks.
/// 
/// # Summary
/// Expected format: `(16-bit Period << 16 | 16-bit Active)`
pub fn unpack_dma_capture(word: u32) -> (u64, u64) {
    let period_ticks = (word >> 16) as u64;
    let active_ticks = (word & 0xFFFF) as u64;
    (period_ticks, active_ticks)
}
