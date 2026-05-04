//! RTIC Example for SMT160 on STM32F103 (Bluepill).
//!
//! This example demonstrates how to use the `Smt160Decoder` in a high-priority 
//! interrupt task to achieve sub-microsecond precision for pulse decoding.
//!
//! ## Hardware Configuration
//! - **MCU**: STM32F103C8T6
//! - **Pin**: PA0 (TIM2 Channel 1)
//! - **Timer**: TIM2 configured for Input Capture with 1µs resolution.
//!
//! ## Wiring
//! - SMT160 VCC -> 5V or 3.3V (Check your sensor variant)
//! - SMT160 GND -> GND
//! - SMT160 OUT -> PA0 (Bluepill)

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use smt160_driver::decoder::Smt160Decoder;
use stm32f1xx_hal::{
    prelude::*,
    pac,
};
use rtic;

#[rtic::app(device = pac, peripherals = true)]
mod app {
    use super::*;
    use fixed::types::I16F16;

    #[shared]
    struct Shared {
        /// The latest verified temperature reading from the SMT160 sensor.
        /// Shared with lower priority tasks or I2C responders.
        current_temp: Option<I16F16>,
    }

    #[local]
    struct Local {
        /// Passive decoder state machine. Operates on timestamps.
        decoder: Smt160Decoder,
        /// TIM2 peripheral for high-precision input capture.
        tim2: pac::TIM2,
        /// Virtual 32-bit timestamp upper half (overflow counter).
        overflows: u32,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        // Initialize clock tree to 72MHz for maximum timing resolution.
        // PCLK1 must be 36MHz (72MHz/2) for standard APB1 timing.
        let mut rcc = rcc.freeze(
            stm32f1xx_hal::rcc::Config::hse(8.MHz())
                .sysclk(72.MHz())
                .pclk1(36.MHz()),
            &mut flash.acr,
        );
        let _clocks = rcc.clocks;

        let mut gpioa = cx.device.GPIOA.split(&mut rcc);
        
        // PA0 (TIM2_CH1) as floating input for the sensor signal.
        let _pa0 = gpioa.pa0.into_floating_input(&mut gpioa.crl);

        let tim2 = cx.device.TIM2;
        
        // --- Timer Configuration ---
        // PSC = 72 - 1 = 71. Since TIM2 is on APB1, it receives 72MHz (36MHz * 2).
        // This gives us exactly 1 tick per microsecond.
        tim2.psc().write(|w| unsafe { w.psc().bits(71) });
        tim2.arr().write(|w| unsafe { w.arr().bits(0xFFFF) });

        // CC1 setup: 
        // 1. Map Channel 1 to TI1.
        // 2. Enable capture on Rising Edge (default).
        tim2.ccmr1_input().modify(|_, w| w.cc1s().ti1());
        tim2.ccer().modify(|_, w| w.cc1e().set_bit().cc1p().clear_bit());
        
        // Enable CC1 capture interrupt and Update (overflow) interrupt.
        tim2.dier().modify(|_, w| w.cc1ie().set_bit().uie().set_bit());
        
        // Start the timer counter.
        tim2.cr1().modify(|_, w| w.cen().set_bit());

        (
            Shared { current_temp: None },
            Local {
                decoder: Smt160Decoder::new(),
                tim2,
                overflows: 0,
            },
            init::Monotonics(),
        )
    }

    /// TIM2 Interrupt Handler: Processes both overflows and capture events.
    ///
    /// This task runs at hardware priority to minimize jitter in timestamp capture.
    #[task(binds = TIM2, local = [decoder, tim2, overflows], shared = [current_temp])]
    fn tim2_irq(mut cx: tim2_irq::Context) {
        let tim2 = cx.local.tim2;
        let sr = tim2.sr().read();

        // 1. Handle Counter Overflow
        // We track the upper 16 bits of our virtual 32-bit timestamp.
        if sr.uif().bit_is_set() {
            tim2.sr().modify(|_, w| w.uif().clear_bit());
            *cx.local.overflows = cx.local.overflows.wrapping_add(1);
        }

        // 2. Handle Input Capture Event
        if sr.cc1if().bit_is_set() {
            let capture = tim2.ccr1().read().ccr().bits();
            
            // Construct virtual 32-bit timestamp in microseconds.
            // Formula: (overflows * 2^16) + capture_ticks
            let timestamp_us = ((*cx.local.overflows as u64) << 16) | (capture as u64);
            
            // Resolve current edge polarity from the CC1P bit.
            let is_rising = tim2.ccer().read().cc1p().bit_is_clear();
            
            // Push the edge to the decoder state machine.
            // Returns Ok(Some(temp)) only when stability and jitter checks pass.
            match cx.local.decoder.push_edge(is_rising, timestamp_us) {
                Ok(Some(temp)) => {
                    // Update shared state for other tasks.
                    cx.shared.current_temp.lock(|t| *t = Some(temp));
                }
                _ => {} // Intermediate pulses or validation failures.
            }

            // Toggle capture polarity to catch the next opposite edge (Falling <-> Rising).
            tim2.ccer().modify(|_, w| w.cc1p().bit(!is_rising));
        }
    }

    /// Idle task: Enters low-power sleep while waiting for interrupts.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}

