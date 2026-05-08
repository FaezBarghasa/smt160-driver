//! STM32F1 DMA & Timer Abstraction for Zero-Jitter Capture
//!
//! This module completely encapsulates the complex PWM-Input Slave-Mode Timer
//! and DMA Burst setup required to achieve < 0.05°C absolute precision.
//!
//! It relies on the `stm32f1xx-hal` PAC for direct register access and utilizes
//! the HAL's standard DMA types to seamlessly integrate into modern RTIC 2.1 applications.

use crate::error::Smt160Error;
use stm32f1xx_hal::rcc::Clocks;

/// Validates that the APB1 clock is running at least at 8 MHz.
/// This is the absolute minimum resolution (125ns/tick) required to
/// guarantee the 0.05°C accuracy specification. 72 MHz (13.8ns/tick) is preferred.
pub fn validate_clocks(clocks: &Clocks) -> Result<(), Smt160Error> {
    // We check pclk1 because TIM2, TIM3, TIM4 reside on the APB1 bus.
    // stm32f1xx-hal uses `fugit` for rate, so .to_Hz() returns the u32 value.
    let pclk1_hz = clocks.pclk1().to_Hz();
    if pclk1_hz < 8_000_000 {
        Err(Smt160Error::ClockTooSlow)
    } else {
        Ok(())
    }
}

/// Trait representing an STM32F1 Timer capable of advanced PWM Input + DMA Burst.
/// This trait isolates the register writes from the user API.
pub trait Smt160TimerInstance {
    /// Configures the timer in Slave Reset Mode, capturing Rising on CC1 and Falling on CC2.
    fn setup_pwm_input(&self);
    /// Returns the physical address of the Timer's DMA Burst (DMAR) register.
    fn dmar_address(&self) -> u32;
}

/// Trait representing a specific DMA Channel mapped to a Timer's CC1 event.
/// By implementing this on the `stm32f1xx-hal` DMA channel types, we allow users
/// to pass standard HAL channels (e.g., `dma1.5`) directly into our driver.
pub trait Smt160DmaChannelInstance {
    /// Configures the DMA channel for circular 32-bit transfers from the Timer DMAR to RAM.
    fn setup_circular_transfer(&self, periph_addr: u32, mem_addr: u32, length: u16);
    /// Clears the Transfer Complete Interrupt Flag (TCIF).
    fn clear_transfer_complete_flag(&self);
    /// Clears the Half Transfer Interrupt Flag (HTIF).
    fn clear_half_transfer_flag(&self);
    /// Checks if the Half Transfer hardware flag is set.
    fn is_half_transfer(&self) -> bool;
    /// Checks if the Transfer Complete hardware flag is set.
    fn is_transfer_complete(&self) -> bool;
}

// ============================================================================
// TIMER MACRO: Encapsulating the PAC register configurations
// ============================================================================

macro_rules! impl_smt160_timer {
    ($TIMX:ident) => {
        impl Smt160TimerInstance for stm32f1xx_hal::pac::$TIMX {
            fn setup_pwm_input(&self) {
                // 1. Disable timer during configuration
                self.cr1.modify(|_, w| w.cen().clear_bit());

                // 2. Map TI1 to both CC1 (Rising) and CC2 (Falling)
                // We use the safe stm32f1xx-hal PAC enum variants for PWM input.
                self.ccmr1_input().modify(|_, w| {
                    w.cc1s()
                        .ti1() // CC1 channel mapped on TI1
                        .cc2s()
                        .ti1() // CC2 channel mapped on TI1
                });

                // 3. Set Polarities and Enable Captures
                self.ccer.modify(|_, w| {
                    w.cc1p()
                        .clear_bit() // CC1 captures Rising edge (Period End / Reset)
                        .cc2p()
                        .set_bit() // CC2 captures Falling edge (Active Time End)
                        .cc1e()
                        .set_bit() // Enable CC1 capture
                        .cc2e()
                        .set_bit() // Enable CC2 capture
                });

                // 4. Configure Slave Mode Reset (Zero-Jitter Hardware Reset)
                // We use the safe PAC enum variants for exact synchronization.
                self.smcr.modify(|_, w| {
                    w.ts()
                        .ti1fp1() // Filtered Timer Input 1 (TI1FP1)
                        .sms()
                        .reset_mode() // Reset Mode: Timer CNT=0 on TI1 rising edge
                });

                // 5. Configure DMA Burst mapping (DMAR)
                // Offset for CCR1 is 0x34. DBA = 0x34 >> 2 = 13.
                // DBL = 1 means 2 transfers per burst request (CCR1, then CCR2).
                // SAFETY: DBA and DBL are within valid 5-bit hardware ranges.
                self.dcr
                    .modify(|_, w| unsafe { w.dba().bits(13).dbl().bits(1) });

                // 6. Enable DMA request exclusively on CC1 (Rising Edge)
                self.dier.modify(|_, w| w.cc1de().set_bit());

                // 7. Start the Timer
                self.cr1.modify(|_, w| w.cen().set_bit());
            }

            fn dmar_address(&self) -> u32 {
                &self.dmar as *const _ as u32
            }
        }
    };
}

// Map the hardware timers capable of running this configuration.
impl_smt160_timer!(TIM2);
impl_smt160_timer!(TIM3);
impl_smt160_timer!(TIM4);

// ============================================================================
// DMA MACRO: Lock-Free Circular Double Buffering
// ============================================================================

macro_rules! impl_smt160_dma_channel {
    ($dma_mod:ident, $CX:ident, $DMAX_PAC:ident, $chX:ident, $tcif:ident, $htif:ident, $ctcif:ident, $chtif:ident) => {
        // Implement on the HAL's specific DMA channel types (e.g., stm32f1xx_hal::dma::dma1::C5)
        impl Smt160DmaChannelInstance for stm32f1xx_hal::dma::$dma_mod::$CX {
            fn setup_circular_transfer(&self, periph_addr: u32, mem_addr: u32, length: u16) {
                // SAFETY: We safely derive the PAC pointer from the global DMA instance.
                let dma = unsafe { &*stm32f1xx_hal::pac::$DMAX_PAC::ptr() };
                let ch = &dma.$chX;

                // Disable channel before modifying addresses.
                ch.ccr.modify(|_, w| w.en().clear_bit());

                // Set peripheral, memory addresses and transfer length (4 words)
                // SAFETY: The provided addresses must be valid static memory or peripheral bounds.
                ch.cpar.write(|w| unsafe { w.bits(periph_addr) });
                ch.cmar.write(|w| unsafe { w.bits(mem_addr) });
                ch.cndtr.write(|w| unsafe { w.bits(length as u32) });

                // Configure: Circular mode, 32-bit sizes, Memory increment, Interrupts
                ch.ccr.modify(|_, w| {
                    w.mem2mem()
                        .clear_bit()
                        .pl()
                        .high() // High priority for real-time sensor
                        .msize()
                        .bits32() // Memory size 32-bit (u32 array)
                        .psize()
                        .bits32() // Peripheral size 32-bit (CCR registers)
                        .minc()
                        .set_bit() // Increment memory (buf[0] -> buf[1])
                        .pinc()
                        .clear_bit() // Fixed peripheral (DMAR automatically handles offset internally)
                        .circ()
                        .set_bit() // Circular mode (Double buffering)
                        .dir()
                        .clear_bit() // Read from peripheral, write to RAM
                        .tcie()
                        .set_bit() // Enable Transfer Complete Interrupt
                        .htie()
                        .set_bit() // Enable Half Transfer Interrupt
                        .en()
                        .set_bit() // Start listening for triggers
                });
            }

            fn clear_transfer_complete_flag(&self) {
                let dma = unsafe { &*stm32f1xx_hal::pac::$DMAX_PAC::ptr() };
                dma.ifcr.write(|w| w.$ctcif().set_bit());
            }

            fn clear_half_transfer_flag(&self) {
                let dma = unsafe { &*stm32f1xx_hal::pac::$DMAX_PAC::ptr() };
                dma.ifcr.write(|w| w.$chtif().set_bit());
            }

            fn is_half_transfer(&self) -> bool {
                let dma = unsafe { &*stm32f1xx_hal::pac::$DMAX_PAC::ptr() };
                dma.isr.read().$htif().bit_is_set()
            }

            fn is_transfer_complete(&self) -> bool {
                let dma = unsafe { &*stm32f1xx_hal::pac::$DMAX_PAC::ptr() };
                dma.isr.read().$tcif().bit_is_set()
            }
        }
    };
}

// Map the specific DMA channels connected to the TIMx_CH1 triggers.
// RM0008 Table 78: DMA1 requests for STM32F103
// TIM2_CH1 -> DMA1_Channel5
// TIM3_CH1 -> DMA1_Channel6
// TIM4_CH1 -> DMA1_Channel1
impl_smt160_dma_channel!(dma1, C5, DMA1, ch5, tcif5, htif5, ctcif5, chtif5);
impl_smt160_dma_channel!(dma1, C6, DMA1, ch6, tcif6, htif6, ctcif6, chtif6);
impl_smt160_dma_channel!(dma1, C1, DMA1, ch1, tcif1, htif1, ctcif1, chtif1);
