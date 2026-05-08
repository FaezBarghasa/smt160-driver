//! Hardware-Accelerated Capture Driver for STM32F1.
//! 
//! This module implements high-precision temperature sensing using the STM32's 
//! PWM Input mode. By utilizing hardware capture channels and slave-reset logic, 
//! we eliminate interrupt latency jitter.

use crate::decoder::Smt160Decoder;
use crate::{Reading, Smt160Error};
use core::sync::atomic::{AtomicU32, Ordering};
use stm32f1xx_hal::pac::TIM2;

/// Global overflow counter for 48-bit virtual timer extension.
/// Incremented in the Timer Update ISR.
static OVERFLOW_COUNT: AtomicU32 = AtomicU32::new(0);

/// Hardware-accelerated capture driver for SMT160 on STM32F1.
///
/// # Architecture
/// This implementation uses the "PWM Input" mode of the STM32 timers, where 
/// CCR1 captures the Period and CCR2 captures the High Time. This is 
/// performed in hardware, eliminating interrupt latency jitter.
///
/// # Usage Example
/// ```
/// use smt160_driver::stm32f1::Smt160Capture;
/// let capture = Smt160Capture::new_tim2(dp.TIM2, decoder);
/// ```
pub struct Smt160Capture<TIM> {
    tim: TIM,
    decoder: Smt160Decoder,
}

impl Smt160Capture<TIM2> {
    /// Initializes TIM2 in PWM Input mode on PA0.
    /// 
    /// This requires:
    /// - PA0 configured as Floating Input.
    /// - TIM2 clock enabled.
    pub fn new_tim2(tim: TIM2, decoder: Smt160Decoder) -> Self {
        // Setup TIM2 registers for PWM Input on TI1 (PA0)
        // 1. Configure CC1 as input on TI1
        tim.ccmr1_input().modify(|_, w| unsafe { w.cc1s().bits(0b01) });
        // 2. Configure CC2 as input on TI1
        tim.ccmr1_input().modify(|_, w| unsafe { w.cc2s().bits(0b10) });
        // 3. Set CC1 to capture on rising edge (default)
        tim.ccer().modify(|_, w| w.cc1p().clear_bit());
        // 4. Set CC2 to capture on falling edge
        tim.ccer().modify(|_, w| w.cc2p().set_bit());
        // 5. Enable CC1 and CC2 capture
        tim.ccer().modify(|_, w| w.cc1e().set_bit().cc2e().set_bit());
        // 6. Configure Slave Mode: Reset on TI1FP1
        tim.smcr().modify(|_, w| unsafe { 
            w.sms().bits(0b100) // Reset Mode
             .ts().bits(0b101)  // TI1FP1
        });
        // 7. Enable Update Interrupt for overflow stitching
        tim.dier().modify(|_, w| w.uie().set_bit());
        // 8. Enable Counter
        tim.cr1().modify(|_, w| w.cen().set_bit());

        Self { tim, decoder }
    }

    /// Handles the Timer Update (Overflow) interrupt.
    /// Must be called from the TIM2 ISR.
    pub fn handle_overflow_isr() {
        OVERFLOW_COUNT.fetch_add(1, Ordering::SeqCst);
    }

    /// Processes a captured cycle and returns a reading if available.
    /// 
    /// This should be called from the TIM2 Capture ISR (triggered by CC1).
    pub fn handle_capture_isr(&mut self) -> Result<Option<Reading>, Smt160Error> {
        // In Slave-Reset mode:
        // CCR1 = Period (in ticks)
        // CCR2 = High Time (in ticks)
        
        let period_raw = self.tim.ccr1().read().bits() as u64;
        let high_raw = self.tim.ccr2().read().bits() as u64;
        
        // Overflow stitching:
        // Since the counter resets on every rising edge (Slave-Reset), 
        // the OVERFLOW_COUNT incremented in UIE represents wraps 
        // since the LAST rising edge.
        let overflows = OVERFLOW_COUNT.swap(0, Ordering::SeqCst) as u64;
        
        // The 48-bit timestamp for the current rising edge (end of period)
        // is the sum of overflows and the current capture value.
        let period_ticks = period_raw + (overflows << 16);
        
        // High time (CC2) is captured on the falling edge. 
        // In Slave-Reset mode, the counter doesn't reset on CC2, only on CC1.
        // So CCR2 is the ticks from the start of the cycle to the falling edge.
        let high_ticks = high_raw;

        // We push a "fake" sequence to the decoder to use its math.
        // T0 (Rise) = 0
        // T1 (Fall) = high_ticks
        // T2 (Rise) = period_ticks
        self.decoder.reset_state();
        self.decoder.push_edge(true, 0)?;
        self.decoder.push_edge(false, high_ticks)?;
        self.decoder.push_edge(true, period_ticks)
    }

    /// Returns the current 48-bit virtual timestamp.
    pub fn get_timestamp(&self) -> u64 {
        let overflows = OVERFLOW_COUNT.load(Ordering::SeqCst) as u64;
        let cnt = self.tim.cnt().read().bits() as u64;
        (overflows << 16) | cnt
    }
}

/// RTIC 2.1 Integration for SMT160.
pub trait Smt160Monotonic {
    fn now(&self) -> u64;
}

impl Smt160Monotonic for Smt160Capture<TIM2> {
    fn now(&self) -> u64 {
        self.get_timestamp()
    }
}
