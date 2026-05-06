//! STM32F1 Flash Persistence for Calibration Data.
//! 
//! Target: STM32F103C8 (64KB Flash)
//! Page 63: 0x0800_FC00 - 0x0800_FFFF (1KB)

use crate::calibration::Calibration;
use fixed::types::I16F16;
use stm32f1xx_hal::pac::FLASH;

/// Flash persistence helper for STM32F103.
pub struct Smt160Flash;

impl Smt160Flash {
    pub const PAGE_START: u32 = 0x0800_FC00;
    pub const MAGIC: u32 = 0x534D5431; // "SMT1"

    /// Loads calibration data from the fixed flash page.
    pub fn load() -> Option<Calibration> {
        let addr = Self::PAGE_START as *const u32;
        unsafe {
            if core::ptr::read_volatile(addr) != Self::MAGIC {
                return None;
            }

            let m_bits = core::ptr::read_volatile(addr.add(1)) as i32;
            let o_bits = core::ptr::read_volatile(addr.add(2)) as i32;
            let crc_stored = (core::ptr::read_volatile(addr.add(3)) & 0xFF) as u8;

            let cal = Calibration {
                multiplier: I16F16::from_bits(m_bits),
                offset: I16F16::from_bits(o_bits),
                p1_raw: None,
                p2_raw: None,
            };

            if cal.crc8() == crc_stored {
                Some(cal)
            } else {
                None
            }
        }
    }

    /// Saves calibration data to the fixed flash page.
    /// 
    /// # Safety
    /// This method uses unsafe register access to perform page erase and program.
    /// It should only be called during a calibration procedure.
    pub fn save(flash: &mut FLASH, cal: &Calibration) -> Result<(), ()> {
        let m_bits = cal.multiplier.to_bits() as u32;
        let o_bits = cal.offset.to_bits() as u32;
        let crc = cal.crc8() as u32;

        unsafe {
            // 1. Unlock Flash
            flash.keyr().write(|w| w.key().bits(0x45670123));
            flash.keyr().write(|w| w.key().bits(0xCDEF89AB));

            // 2. Erase Page 63
            while flash.sr().read().bsy().bit_is_set() {}
            flash.cr().modify(|_, w| w.per().set_bit());
            flash.ar().write(|w| w.far().bits(Self::PAGE_START));
            flash.cr().modify(|_, w| w.strt().set_bit());
            while flash.sr().read().bsy().bit_is_set() {}
            flash.cr().modify(|_, w| w.per().clear_bit());

            // 3. Program Words
            let data = [Self::MAGIC, m_bits, o_bits, crc];
            for (i, &word) in data.iter().enumerate() {
                flash.cr().modify(|_, w| w.pg().set_bit());
                let addr = (Self::PAGE_START + (i as u32 * 4)) as *mut u16;
                
                // STM32F1 programs 16 bits at a time
                core::ptr::write_volatile(addr, word as u16);
                while flash.sr().read().bsy().bit_is_set() {}
                
                core::ptr::write_volatile(addr.add(1), (word >> 16) as u16);
                while flash.sr().read().bsy().bit_is_set() {}
                
                flash.cr().modify(|_, w| w.pg().clear_bit());
            }

            // 4. Lock Flash
            flash.cr().modify(|_, w| w.lock().set_bit());
        }

        Ok(())
    }
}
