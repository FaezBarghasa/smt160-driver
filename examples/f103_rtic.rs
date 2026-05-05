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
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        let rcc = rcc.freeze(
            stm32f1xx_hal::rcc::Config::hse(8.MHz())
                .sysclk(72.MHz())
                .pclk1(36.MHz()),
            &mut flash.acr,
        );

        let mut gpioa = cx.device.GPIOA.split(&mut rcc);
        let _pa0 = gpioa.pa0.into_floating_input(&mut gpioa.crl);

        let tim2 = cx.device.TIM2;
        
        // --- High-Precision PWM Input Configuration ---
        // 1. Set Prescaler to 72 (1 tick = 1 microsecond @ 72MHz)
        tim2.psc().write(|w| unsafe { w.psc().bits(71) });
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
        tim2.smcr().modify(|_, w| unsafe { w.sms().bits(0b100) });

        // 8. Enable Captures and Interrupts
        tim2.ccer().modify(|_, w| w.cc1e().set_bit().cc2e().set_bit());
        tim2.dier().modify(|_, w| w.cc1ie().set_bit());

        // 9. Start Counter
        tim2.cr1().modify(|_, w| w.cen().set_bit());

        (
            Shared { current_temp: None },
            Local {
                decoder: Smt160Decoder::new(),
                tim2,
            },
            init::Monotonics(),
        )
    }

    #[task(binds = TIM2, local = [decoder, tim2], shared = [current_temp])]
    fn tim2_irq(mut cx: tim2_irq::Context) {
        let tim2 = cx.local.tim2;
        
        if tim2.sr().read().cc1if().bit_is_set() {
            // Read CCR1 (Period) and CCR2 (High Time)
            // Hardware resets the counter on Rising Edge, so these are absolute values.
            let period = tim2.ccr1().read().ccr().bits() as u64;
            let high_time = tim2.ccr2().read().ccr().bits() as u64;

            // In PWM input mode, we get both Rise1 and Fall in one interrupt (on the next Rise2).
            // To work with Smt160Decoder, we can simulate the edges or just use the math.
            // Since Smt160Decoder::push_edge expects individual edges, we feed it:
            // 1. The previous Rise1 (implied by hardware reset)
            // 2. The Fall (high_time later)
            // 3. The current Rise2 (period later)

            // Actually, we can just push edges with artificial timestamps relative to a base.
            let base = 0;
            let _ = cx.local.decoder.push_edge(true, base);
            let _ = cx.local.decoder.push_edge(false, base + high_time);
            match cx.local.decoder.push_edge(true, base + period) {
                Ok(Some(temp)) => {
                    cx.shared.current_temp.lock(|t| *t = Some(temp));
                }
                _ => {}
            }
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}
