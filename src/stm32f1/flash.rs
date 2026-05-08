//! STM32F1xx Flash Persistence Backend for Calibration Profiles.

use embedded_storage::{ReadStorage, Storage};
use stm32f1xx_hal::pac::FLASH;
use critical_section;

/// Persistent storage implementation utilizing the STM32F1 internal Flash memory.
/// 
/// # Architecture
/// This implementation targets a fixed 1KB page at the end of the 64KB Flash 
/// memory space (Page 63). All write operations are protected by `critical-section` 
/// to ensure atomicity and prevent Bus Faults.
///
/// # Usage Example
/// ```
/// use smt160_driver::stm32f1::Smt160FlashBackend;
/// let mut storage = Smt160FlashBackend::new(&mut dp.FLASH);
/// ```
pub struct Smt160FlashBackend<'a> {
    flash_peripheral: &'a mut FLASH,
}

impl<'a> Smt160FlashBackend<'a> {
    /// The starting memory address of the dedicated calibration Flash page (Page 63).
    pub const CALIBRATION_PAGE_START_ADDRESS: u32 = 0x0800_FC00;

    /// Creates a new Flash storage backend with the provided peripheral.
    ///
    /// # Panics
    /// This function does not panic.
    pub fn new(flash_peripheral: &'a mut FLASH) -> Self {
        Self { flash_peripheral }
    }
}

impl<'a> ReadStorage for Smt160FlashBackend<'a> {
    type Error = core::convert::Infallible;

    /// Reads a sequence of bytes from Flash memory starting at the specified offset.
    fn read(&mut self, memory_offset: u32, data_buffer: &mut [u8]) -> Result<(), Self::Error> {
        let source_address = (Self::CALIBRATION_PAGE_START_ADDRESS + memory_offset) as *const u8;
        for (i, byte) in data_buffer.iter_mut().enumerate() {
            unsafe {
                *byte = core::ptr::read_volatile(source_address.add(i));
            }
        }
        Ok(())
    }

    /// Returns the total storage capacity of the dedicated calibration page.
    fn capacity(&self) -> usize {
        1024 // 1KB Page
    }
}

impl<'a> Storage for Smt160FlashBackend<'a> {
    /// Writes a sequence of bytes to Flash memory.
    /// 
    /// # Safety
    /// This method performs an atomic Page Erase followed by a Half-Word Program 
    /// sequence within a `critical-section`.
    fn write(&mut self, memory_offset: u32, data_buffer: &[u8]) -> Result<(), Self::Error> {
        critical_section::with(|_| {
            unsafe {
                // 1. Unlock Flash Controller
                self.flash_peripheral.keyr().write(|w| w.key().bits(0x45670123));
                self.flash_peripheral.keyr().write(|w| w.key().bits(0xCDEF89AB));

                // 2. Erase the calibration page
                while self.flash_peripheral.sr().read().bsy().bit_is_set() {}
                self.flash_peripheral.cr().modify(|_, w| w.per().set_bit());
                self.flash_peripheral.ar().write(|w| w.far().bits(Self::CALIBRATION_PAGE_START_ADDRESS));
                self.flash_peripheral.cr().modify(|_, w| w.strt().set_bit());
                while self.flash_peripheral.sr().read().bsy().bit_is_set() {}
                self.flash_peripheral.cr().modify(|_, w| w.per().clear_bit());

                // 3. Program Data (16-bit Half-Word writes)
                for i in (0..data_buffer.len()).step_by(2) {
                    self.flash_peripheral.cr().modify(|_, w| w.pg().set_bit());
                    let half_word = if i + 1 < data_buffer.len() {
                        u16::from_le_bytes([data_buffer[i], data_buffer[i+1]])
                    } else {
                        u16::from_le_bytes([data_buffer[i], 0xFF])
                    };
                    
                    let target_address = (Self::CALIBRATION_PAGE_START_ADDRESS + memory_offset + i as u32) as *mut u16;
                    core::ptr::write_volatile(target_address, half_word);
                    while self.flash_peripheral.sr().read().bsy().bit_is_set() {}
                    self.flash_peripheral.cr().modify(|_, w| w.pg().clear_bit());
                }

                // 4. Lock Flash Controller
                self.flash_peripheral.cr().modify(|_, w| w.lock().set_bit());
            }
        });

        Ok(())
    }
}
