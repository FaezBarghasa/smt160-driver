//! STM32F1 DMA & Timer Abstraction for Zero-Jitter Capture
//!
//! This module completely encapsulates the complex PWM-Input Slave-Mode Timer
//! and DMA Burst setup required to achieve < 0.05°C absolute precision.
//!
//! It relies on the `stm32f1xx-hal` PAC for direct register access and utilizes
//! the HAL's standard DMA types to seamlessly integrate into modern RTIC 2.1 applications.

use crate::error::Smt160Error;
use stm32f1xx_hal::pac;
use stm32f1xx_hal::rcc::Clocks;

/// Validates that the APB1 clock is running at least at 8 MHz.
/// This is the absolute minimum resolution (125ns/tick) required to
/// guarantee the 0.05°C accuracy specification. 72 MHz (13.8ns/tick) is preferred.
pub fn validate_clocks(clocks: &Clocks) -> Result<(), Smt160Error> {
    let pclk1_hz = clocks.pclk1().to_Hz();
    if pclk1_hz < 8_000_000 {
        Err(Smt160Error::ClockTooSlow)
    } else {
        Ok(())
    }
}

/// Trait representing an STM32F1 Timer capable of advanced PWM Input + DMA Burst.
pub trait Smt160TimerInstance {
    /// Configures the timer in Slave Reset Mode, capturing Rising on CC1 and Falling on CC2.
    fn setup_pwm_input(&self);
    /// Configures the DMA Burst (DMAR) to fetch CCR1 and CCR2 on every CC1 event.
    fn setup_dma_burst(&self);
    /// Returns the physical address of the Timer's DMA Burst (DMAR) register.
    fn dmar_address(&self) -> u32;
    /// Disables and resets the hardware to a known good state.
    fn reset_hardware(&self);
}

/// Trait representing a specific DMA Channel mapped to a Timer's CC1 event.
pub trait Smt160DmaChannel {
    /// Configures the DMA channel for circular 32-bit transfers from the Timer DMAR to RAM.
    /// 
    /// # Safety
    /// `memory_addr` must point to a valid, pinned buffer of at least `len` words.
    unsafe fn setup_circular_capture(&self, peripheral_addr: u32, memory_addr: *mut u32, len: u16);
    /// Clears all interrupt flags (Transfer Complete, Half Transfer, Error).
    fn clear_interrupt_flags(&self);
    /// Checks if the Half Transfer hardware flag is set.
    fn is_half_transfer(&self) -> bool;
    /// Checks if the Transfer Complete hardware flag is set.
    fn is_transfer_complete(&self) -> bool;
    /// Disables the DMA channel.
    fn disable(&self);
}

// ============================================================================
// TIMER MACRO
// ============================================================================

macro_rules! impl_smt160_timer {
    ($TIMX:ident) => {
        impl Smt160TimerInstance for pac::$TIMX {
            fn setup_pwm_input(&self) {
                // 1. Map TI1 to both CC1 (Rising) and CC2 (Falling)
                self.ccmr1_input().modify(|_, w| {
                    w.cc1s().ti1()
                     .cc2s().ti1()
                });

                self.ccer().modify(|_, w| {
                    w.cc1p().clear_bit() // CC1 captures Rising
                     .cc2p().set_bit()   // CC2 captures Falling
                     .cc1e().set_bit()   // Enable CC1
                     .cc2e().set_bit()   // Enable CC2
                });

                self.smcr().modify(|_, w| {
                    w.ts().ti1fp1()
                     .sms().reset_mode()
                });

                self.cr1().modify(|_, w| w.cen().set_bit());
            }

            fn setup_dma_burst(&self) {
                self.dcr().modify(|_, w| unsafe { w.dba().bits(13).dbl().bits(1) });
                self.dier().modify(|_, w| w.cc1de().set_bit());
            }

            fn dmar_address(&self) -> u32 {
                &self.dmar() as *const _ as u32
            }

            fn reset_hardware(&self) {
                self.cr1().modify(|_, w| w.cen().clear_bit());
                self.dier().modify(|_, w| w.cc1de().clear_bit());
                self.sr().write(|w| unsafe { w.bits(0) });
            }
        }
    };
}

impl_smt160_timer!(TIM2);
impl_smt160_timer!(TIM3);
impl_smt160_timer!(TIM4);

// ============================================================================
// DMA MACRO
// ============================================================================

macro_rules! impl_smt160_dma {
    ($($CH:ident, $field:ident, $offset:expr),+) => {
        $(
            impl Smt160DmaChannel for stm32f1xx_hal::dma::dma1::$CH {
                unsafe fn setup_circular_capture(&self, peripheral_addr: u32, memory_addr: *mut u32, len: u16) {
                    let dma1 = unsafe { &*pac::DMA1::ptr() };
                    let ch = dma1.$field();

                    // Disable before configuration
                    ch.cr().modify(|_, w| w.en().clear_bit());

                    ch.par().write(|w| unsafe { w.pa().bits(peripheral_addr) });
                    ch.mar().write(|w| unsafe { w.ma().bits(memory_addr as u32) });
                    ch.ndtr().write(|w| unsafe { w.ndt().bits(len) });

                    // CR: 32-bit MSIZE/PSIZE, MINC, CIRC, HTIE, TCIE, EN
                    ch.cr().modify(|_, w| unsafe {
                        w.msize().bits(0b10);
                        w.psize().bits(0b10);
                        w.minc().set_bit();
                        w.circ().set_bit();
                        w.htie().set_bit();
                        w.tcie().set_bit();
                        w.en().set_bit()
                    });
                }

                fn clear_interrupt_flags(&self) {
                    let dma1 = unsafe { &*pac::DMA1::ptr() };
                    dma1.ifcr().write(|w| unsafe { w.bits(0xF << ($offset * 4)) });
                }

                fn is_half_transfer(&self) -> bool {
                    let dma1 = unsafe { &*pac::DMA1::ptr() };
                    (dma1.isr().read().bits() >> ($offset * 4 + 2)) & 1 != 0
                }

                fn is_transfer_complete(&self) -> bool {
                    let dma1 = unsafe { &*pac::DMA1::ptr() };
                    (dma1.isr().read().bits() >> ($offset * 4 + 1)) & 1 != 0
                }

                fn disable(&self) {
                    let dma1 = unsafe { &*pac::DMA1::ptr() };
                    dma1.$field().cr().modify(|_, w| w.en().clear_bit());
                }
            }
        )+
    }
}

impl_smt160_dma!(C1, ch1, 0, C5, ch5, 4, C6, ch6, 5);
