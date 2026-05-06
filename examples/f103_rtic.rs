//! High-Accuracy RTIC v2.1 Example for SMT160 on STM32F103 (Bluepill).
//!
//! This example uses the STM32 "PWM Input" hardware mode for zero-jitter
//! period and active-time measurement, refactored for RTIC v2.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use smt160_driver::decoder::Smt160Decoder;
use stm32f1xx_hal::{
    prelude::*,
    pac,
};

#[rtic::app(device = pac, dispatchers = [EXTI0])]
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
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();
        let mut rcc = rcc.freeze(
            stm32f1xx_hal::rcc::Config::hse(8.MHz())
                .sysclk(72.MHz())
                .pclk1(36.MHz()),
            &mut flash.acr,
        );

        let mut gpioa = cx.device.GPIOA.split(&mut rcc.apb2);
        let _pa0 = gpioa.pa0.into_floating_input(&mut gpioa.crl);

        let tim2 = cx.device.TIM2;
        
        // --- High-Precision PWM Input Configuration ---
        tim2.psc().write(|w| unsafe { w.psc().bits(0) });
        tim2.arr().write(|w| unsafe { w.arr().bits(0xFFFF) });

        tim2.ccmr1_input().modify(|_, w| w.cc1s().ti1());
        tim2.ccmr1_input().modify(|_, w| w.cc2s().ti1());
        tim2.ccer().modify(|_, w| w.cc1p().clear_bit());
        tim2.ccer().modify(|_, w| w.cc2p().set_bit());

        tim2.smcr().modify(|_, w| unsafe { w.ts().bits(0b101) });
        tim2.smcr().modify(|_, w| unsafe { w.sms().bits(0b100) });

        tim2.ccer().modify(|_, w| w.cc1e().set_bit().cc2e().set_bit());
        tim2.dier().modify(|_, w| w.cc1ie().set_bit().uie().set_bit());
        tim2.cr1().modify(|_, w| w.cen().set_bit());

        (
            Shared { current_temp: None },
            Local {
                decoder: Smt160Decoder::from_clocks(&rcc.clocks),
                tim2,
                overflows: 0,
                monotonic_base: 0,
            },
        )
    }

    #[task(binds = TIM2, local = [decoder, tim2, overflows, monotonic_base], shared = [current_temp])]
    fn tim2_irq(mut cx: tim2_irq::Context) {
        let tim2 = cx.local.tim2;
        let sr = tim2.sr().read();

        if sr.uif().bit_is_set() {
            tim2.sr().modify(|_, w| w.uif().clear_bit());
            *cx.local.overflows += 1;
        }

        if sr.cc1if().bit_is_set() {
            let raw_period = tim2.ccr1().read().ccr().bits() as u64;
            let raw_high = tim2.ccr2().read().ccr().bits() as u64;
            let ovf = *cx.local.overflows;
            *cx.local.overflows = 0;

            let period_ticks = (ovf as u64 * 65536) + raw_period;
            let high_ticks = if raw_high > raw_period {
                ((ovf as u64).saturating_sub(1) * 65536) + raw_high
            } else {
                (ovf as u64 * 65536) + raw_high
            };
            
            let base = *cx.local.monotonic_base;
            let _ = cx.local.decoder.push_edge(true, base);
            let _ = cx.local.decoder.push_edge(false, base + high_ticks);
            match cx.local.decoder.push_edge(true, base + period_ticks) {
                Ok(Some(reading)) => {
                    cx.shared.current_temp.lock(|t| *t = Some(reading.value));
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
