//! STM32F1 Specific DMA and Timer HAL implementation.

use crate::error::Smt160Error;
use stm32f1xx_hal::pac;
use stm32f1xx_hal::rcc::Clocks;

/// Validates that the system clocks meet the precision requirements.
///
/// To achieve 0.05°C precision, the timer clock must be at least 8MHz (125ns resolution).
pub fn validate_clocks(clocks: &Clocks) -> Result<(), Smt160Error> {
    // TIM2-4 are on PCLK1.
    let pclk1 = clocks.pclk1().to_Hz();

    // Most STM32F1 timers have a clock multiplier if the APB prescaler is not 1.
    // In many configurations, the timer clock is 2x PCLK1 if APB1 prescaler > 1.
    // We check if PCLK1 is at least 8MHz.
    if pclk1 < 8_000_000 {
        return Err(Smt160Error::ClockTooSlow);
    }

    Ok(())
}

/// Trait defining the requirements for a Timer instance to be used with Smt160.
pub trait Smt160TimerInstance {
    /// Configures the timer for PWM Input Mode with Hardware Reset.
    fn setup_pwm_input(&self);

    /// Returns the address of the DMAR register for DMA Burst transfers.
    fn dmar_address(&self) -> u32;

    /// Configures the Timer DMA Burst settings.
    /// DBA = 13 (CCR1 offset), DBL = 1 (2 sequential transfers: CCR1 then CCR2)
    fn setup_dma_burst(&self);

    /// Resets the timer hardware to a clean state.
    fn reset_hardware(&self);
}

/// Trait for DMA channels used to capture Timer DMAR bursts.
pub trait Smt160DmaChannel {
    /// Configures the DMA channel for circular burst capture.
    ///
    /// # Safety
    /// This function takes a raw pointer to a buffer. The caller must ensure
    /// the buffer is 'static and valid for the duration of the capture.
    unsafe fn setup_circular_capture(&self, peripheral_addr: u32, memory_addr: *mut u32, len: u16);

    /// Clears the Half-Transfer and Transfer-Complete flags.
    fn clear_interrupt_flags(&self);

    /// Checks if a Half-Transfer interrupt occurred.
    fn is_half_transfer(&self) -> bool;

    /// Checks if a Transfer-Complete interrupt occurred.
    fn is_transfer_complete(&self) -> bool;

    /// Disables the DMA channel.
    fn disable(&self);
}

macro_rules! impl_smt160_timer {
    ($($TIM:ident),+) => {
        $(
            impl Smt160TimerInstance for pac::$TIM {
                fn setup_pwm_input(&self) {
                    // 1. Configure CC1 as Input on TI1 (Rising Edge)
                    // 2. Configure CC2 as Input on TI1 (Falling Edge)
                    self.ccmr1_input().modify(|_, w| unsafe {
                        w.cc1s().bits(0b01); // CC1 -> TI1
                        w.cc2s().bits(0b10)  // CC2 -> TI1
                    });

                    // 3. Configure Polarities and Enable Captures
                    self.ccer().modify(|_, w| {
                        w.cc1p().clear_bit();  // Rising
                        w.cc1e().set_bit();    // Enable CC1
                        w.cc2p().set_bit();    // Falling
                        w.cc2e().set_bit()     // Enable CC2
                    });

                    // 4. Configure Slave Mode Control Register (SMCR)
                    // SMS = 100 (Reset Mode): Rising edge on selected trigger resets counter
                    // TS = 101 (Filtered Timer Input 1 - TI1FP1)
                    self.smcr().modify(|_, w| unsafe {
                        w.sms().bits(0b100);
                        w.ts().bits(0b101)
                    });

                    // 5. Enable Counter
                    self.cr1().modify(|_, w| w.cen().set_bit());
                }

                fn dmar_address(&self) -> u32 {
                    self.dmar().as_ptr() as u32
                }

                fn setup_dma_burst(&self) {
                    // DBA = 13 (Offset to CCR1), DBL = 1 (2 sequential transfers: CCR1, CCR2)
                    self.dcr().modify(|_, w| unsafe {
                        w.dba().bits(13);
                        w.dbl().bits(1)
                    });

                    // Enable Capture/Compare 1 DMA request
                    self.dier().modify(|_, w| w.cc1de().set_bit());
                }

                fn reset_hardware(&self) {
                    self.cr1().modify(|_, w| w.cen().clear_bit());
                    self.sr().write(|w| unsafe { w.bits(0) });
                    self.dier().write(|w| unsafe { w.bits(0) });
                }
            }
        )+
    }
}

impl_smt160_timer!(TIM2, TIM3, TIM4);

macro_rules! impl_smt160_dma {
    ($($CH:ident, $field:ident, $offset:expr),+) => {
        $(
            impl Smt160DmaChannel for stm32f1xx_hal::dma::dma1::$CH {
                unsafe fn setup_circular_capture(&self, peripheral_addr: u32, memory_addr: *mut u32, len: u16) {
                    let dma1 = &*pac::DMA1::ptr();
                    let ch = dma1.$field();

                    // Disable before configuration
                    ch.ccr().modify(|_, w| w.en().clear_bit());

                    ch.cpar().write(|w| unsafe { w.pa().bits(peripheral_addr) });
                    ch.cmar().write(|w| unsafe { w.ma().bits(memory_addr as u32) });
                    ch.cndtr().write(|w| unsafe { w.ndt().bits(len) });

                    // CCR:
                    // - MSIZE = 10 (32-bit)
                    // - PSIZE = 10 (32-bit)
                    // - MINC = 1 (Memory increment)
                    // - CIRC = 1 (Circular mode)
                    // - HTIE = 1 (Half Transfer Interrupt Enable)
                    // - TCIE = 1 (Transfer Complete Interrupt Enable)
                    // - EN = 1 (Enable)
                    ch.ccr().modify(|_, w| unsafe {
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
                    dma1.$field().ccr().modify(|_, w| w.en().clear_bit());
                }
            }
        )+
    }
}

// STM32F1 DMA1 offsets: CH1=0, CH2=1, ..., CH5=4, CH6=5, CH7=6
impl_smt160_dma!(C1, ch1, 0, C5, ch5, 4, C6, ch6, 5);
