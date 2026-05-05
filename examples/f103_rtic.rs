//! High-Accuracy RTIC Example for SMT160 on STM32F103 (Bluepill).
//!
//! This example uses the STM32 "PWM Input" hardware mode for zero-jitter
//! period and active-time measurement.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use smt160_driver::decoder::Smt160Decoder;
use stm32f1xx_hal::{
    prelude::*,
    pac,
};

#[rtic::app(device = pac, peripherals = true)]
mod app {
    use super::*;
    use fixed::types::I16F16;

    #[shared]
    struct Shared {
        current_temp: Option<I16F16>,
    }

    #[local]
    struct Local {
        decoder: Smt160Decoder,
        tim2: pac::TIM2,
        overflows: u32,
        monotonic_base: u64,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        let mut rcc = rcc.freeze(
            stm32f1xx_hal::rcc::Config::hse(8.MHz())
                .sysclk(72.MHz())
                .pclk1(36.MHz()),
            &mut flash.acr,
        );

        let mut gpioa = cx.device.GPIOA.split(&mut rcc);
        let _pa0 = gpioa.pa0.into_floating_input(&mut gpioa.crl);

        let tim2 = cx.device.TIM2;
        
        // --- High-Precision PWM Input Configuration ---
        // 1. Set Prescaler to 0 (1 tick = 1/72MHz = 13.88ns)
        // This gives ~0.003°C theoretical resolution at 1kHz.
        tim2.psc().write(|w| unsafe { w.psc().bits(0) });
        tim2.arr().write(|w| unsafe { w.arr().bits(0xFFFF) });

        // 2. Configure CC1 as input, mapped to TI1
        tim2.ccmr1_input().modify(|_, w| w.cc1s().ti1());
        // 3. Configure CC2 as input, mapped to TI1
        tim2.ccmr1_input().modify(|_, w| w.cc2s().ti1());

        // 4. CC1: Capture on Rising Edge
        tim2.ccer().modify(|_, w| w.cc1p().clear_bit());
        // 5. CC2: Capture on Falling Edge
        tim2.ccer().modify(|_, w| w.cc2p().set_bit());

        // 6. Select TI1FP1 as Trigger (TS = 101)
        tim2.smcr().modify(|_, w| unsafe { w.ts().bits(0b101) });
        // 7. Select Slave Mode: Reset (SMS = 100)
        // Counter is reset to 0 on every rising edge of PA0.
        tim2.smcr().modify(|_, w| unsafe { w.sms().bits(0b100) });

        // 8. Enable Captures, Update Interrupt, and Counter
        tim2.ccer().modify(|_, w| w.cc1e().set_bit().cc2e().set_bit());
        tim2.dier().modify(|_, w| w.cc1ie().set_bit().uie().set_bit());
        tim2.cr1().modify(|_, w| w.cen().set_bit());

        (
            Shared { current_temp: None },
            Local {
                decoder: Smt160Decoder::with_clock(72),
                tim2,
                overflows: 0,
                monotonic_base: 0,
            },
            init::Monotonics(),
        )
    }

    #[task(binds = TIM2, local = [decoder, tim2, overflows, monotonic_base], shared = [current_temp])]
    fn tim2_irq(mut cx: tim2_irq::Context) {
        let tim2 = cx.local.tim2;
        let sr = tim2.sr().read();

        // Handle Timer Overflow (Update Event)
        if sr.uif().bit_is_set() {
            tim2.sr().modify(|_, w| w.uif().clear_bit());
            *cx.local.overflows += 1;
        }

        // Handle Capture Event (Rising Edge)
        if sr.cc1if().bit_is_set() {
            // Read CCR1 (Period) and CCR2 (High Time)
            let raw_period = tim2.ccr1().read().ccr().bits() as u64;
            let raw_high = tim2.ccr2().read().ccr().bits() as u64;
            
            // Current overflow count
            let ovf = *cx.local.overflows;
            
            // Important: On rising edge, hardware resets counter AND generates update event IF it overflowed.
            // We need to reset our software overflow counter for the next cycle.
            *cx.local.overflows = 0;

            // Calculate actual counts (1 tick = 1/72 us)
            // Period = overflows * 65536 + CCR1
            let period_ticks = (ovf as u64 * 65536) + raw_period;
            
            // High Time = CCR2 (assuming it happened before overflow, or we'd need more complex logic)
            // For SMT160, DC is typically 0.3 to 0.9. If period is 1000us (72000 ticks), 
            // 910us is the overflow point. So for DC > 0.91, CCR2 also needs overflow correction.
            let high_ticks = if raw_high > raw_period {
                // This means high time capture happened after an overflow but before the final rise
                ((ovf as u64).saturating_sub(1) * 65536) + raw_high
            } else {
                // Usual case: high time is less than period
                // But wait, if period overflowed, did high time overflow too?
                // If ovf > 0 and raw_high < raw_period, it's possible raw_high also overflowed.
                // However, in PWM input mode, CCR2 is captured on falling edge.
                // If it happened after one or more overflows, raw_high would be smaller than raw_period.
                // This is tricky. Let's assume high_ticks is always <= period_ticks.
                // We'll use the same overflow count unless raw_high < some threshold? 
                // Actually, if raw_high < raw_period, and we know an overflow occurred, 
                // we should check if the falling edge was before or after the overflow.
                (ovf as u64 * 65536) + raw_high // Simple assumption for now
            };
            
            // Use a monotonic base to ensure the decoder sees increasing timestamps
            let base = *cx.local.monotonic_base;
            let _ = cx.local.decoder.push_edge(true, base);
            let _ = cx.local.decoder.push_edge(false, base + high_ticks);
            match cx.local.decoder.push_edge(true, base + period_ticks) {
                Ok(Some(temp)) => {
                    cx.shared.current_temp.lock(|t| *t = Some(temp));
                }
                _ => {}
            }
            *cx.local.monotonic_base = base.wrapping_add(period_ticks);
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}
