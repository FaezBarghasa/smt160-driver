//! Production-Grade STM32F1 Managed DMA Driver for SMT160.
//!
//! # Safety
//! This driver assumes exclusive ownership of the provided TIM2 and DMA1 Channel 7.
//! The user must ensure that PA1 (TIM2 CH2) is configured as a Floating Input 
//! or Pull-Up Input before initializing the driver.

use crate::Smt160Error;
use stm32f1xx_hal::pac::{TIM2, DMA1};
use core::sync::atomic::{AtomicBool, Ordering};

/// Size of the circular DMA capture buffer.
pub const DMA_BUFFER_SIZE: usize = 64;

/// Managed Hardware Abstraction for SMT160 on STM32F103.
pub struct Smt160Dma {
    timer: TIM2,
    dma_channel: stm32f1xx_hal::dma::C7,
    buffer: &'static mut [u32; DMA_BUFFER_SIZE],
}

impl Smt160Dma {
    /// Creates a new production-ready SMT160 DMA driver.
    /// 
    /// # Summary
    /// This is a high-level "Managed" entry point that configures TIM2 + DMA1 
    /// for zero-jitter background pulse capture.
    ///
    /// # Errors
    /// Returns `Smt160Error::InvalidConfiguration` if the peripheral clock (PCLK1) 
    /// is insufficient for the $0.05^\circ\text{C}$ accuracy target.
    pub fn new(
        timer: TIM2, 
        dma_channel: stm32f1xx_hal::dma::C7, 
        pclk_mhz: u32,
        buffer: &'static mut [u32; DMA_BUFFER_SIZE]
    ) -> Result<Self, Smt160Error> {
        // Minimum frequency for $0.05^\circ\text{C}$ precision
        if pclk_mhz < 8 {
            return Err(Smt160Error::InvalidConfiguration);
        }

        // 1. Configure TIM2 in PWM Input Mode
        // Using Channel 2 (TI2) as the trigger source
        timer.ccmr1_input().modify(|_, w| unsafe {
            w.cc1s().bits(0b10) // TI2
             .cc2s().bits(0b01) // TI2
        });

        timer.ccer().modify(|_, w| {
            w.cc1p().set_bit()   // Falling (Active)
             .cc2p().clear_bit() // Rising (Period)
             .cc1e().set_bit()   // Enable
             .cc2e().set_bit()   // Enable
        });

        // Slave Mode: Reset on TI2 Rising Edge
        timer.smcr().modify(|_, w| unsafe {
            w.sms().bits(0b100) // Reset Mode
             .ts().bits(0b110)  // TI2FP2
        });

        // 2. Configure DMA Burst Mode (DMAR)
        // We want to transfer CCR1 and CCR2 in one DMA request
        // Base address = CCR1 (offset 0x34 / 4 = 13)
        // Length = 2 registers
        timer.dcr().write(|w| unsafe {
            w.dba().bits(13) // CCR1
             .dbl().bits(1)  // 2 transfers
        });

        // Enable DMA request on CC2 (Rising Edge / Period Complete)
        timer.dier().modify(|_, w| w.cc2de().set_bit());

        // 3. Configure DMA1 Channel 7
        // Source: TIM2_DMAR, Destination: buffer
        // Note: We use the PAC directly here for maximum performance and direct control
        let dma1 = unsafe { &*DMA1::ptr() };
        dma1.ch7.cpadr().write(|w| unsafe { w.bits(timer.dmar().as_ptr() as u32) });
        dma1.ch7.cmar().write(|w| unsafe { w.bits(buffer.as_ptr() as u32) });
        dma1.ch7.cndtr().write(|w| unsafe { w.bits(DMA_BUFFER_SIZE as u32) });

        dma1.ch7.ccr().modify(|_, w| unsafe {
            w.pl().bits(0b10)    // High priority
             .msize().bits(0b10) // 32-bit memory
             .psize().bits(0b10) // 32-bit peripheral
             .minc().set_bit()   // Memory increment
             .circ().set_bit()   // Circular mode
             .en().set_bit()     // Enable
        });

        // Start Timer
        timer.cr1().modify(|_, w| w.cen().set_bit());

        Ok(Self { timer, dma_channel, buffer })
    }

    /// Returns the raw capture buffer for zero-copy processing.
    pub fn get_capture_buffer(&self) -> &[u32] {
        self.buffer
    }

    /// Checks if a new batch of data is ready (Half or Full transfer).
    pub fn poll_status(&self) -> bool {
        let dma1 = unsafe { &*DMA1::ptr() };
        dma1.isr().read().tcif7().bit_is_set() || dma1.isr().read().htif7().bit_is_set()
    }

    /// Clears DMA interrupt flags.
    pub fn clear_flags(&self) {
        let dma1 = unsafe { &*DMA1::ptr() };
        dma1.ifcr().write(|w| w.ctcif7().set_bit().chtif7().set_bit());
    }
}
