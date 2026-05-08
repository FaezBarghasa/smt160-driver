#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use rtic::app;
use stm32f1xx_hal::{
    prelude::*,
    pac,
};
use smt160_driver::decoder::Smt160Decoder;
use smt160_driver::stm32f1::Smt160Capture;

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [USART1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        capture: Smt160Capture<pac::TIM2>,
        count: u32,
        min_dc: i32,
        max_dc: i32,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();
        let mut clocks = rcc.freeze(
            stm32f1xx_hal::rcc::Config::hse(8.MHz())
                .sysclk(72.MHz())
                .pclk1(36.MHz()),
            &mut flash.acr,
        );

        let mut gpioa = cx.device.GPIOA.split(&mut clocks);
        let _pa0 = gpioa.pa0.into_floating_input(&mut gpioa.crl);

        let decoder = Smt160Decoder::new_standalone(72);
        let capture = Smt160Capture::new_tim2(cx.device.TIM2, decoder);

        defmt::info!("SMT160 Jitter Bench Started (72MHz)");

        (Shared {}, Local { 
            capture, 
            count: 0,
            min_dc: i32::MAX,
            max_dc: i32::MIN,
        })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = TIM2, local = [capture, count, min_dc, max_dc])]
    fn on_tim2(cx: on_tim2::Context) {
        let tim2 = unsafe { &*pac::TIM2::ptr() };
        
        // Check for Overflow
        if tim2.sr().read().uif().bit_is_set() {
            tim2.sr().modify(|_, w| w.uif().clear_bit());
            Smt160Capture::<pac::TIM2>::handle_overflow_isr();
        }

        // Check for Capture on CC1 (Rising Edge / End of Period)
        if tim2.sr().read().cc1if().bit_is_set() {
            // Note: Reading CCR1 clears CC1IF
            if let Ok(Some(reading)) = cx.local.capture.handle_capture_isr() {
                *cx.local.count += 1;
                
                let val_bits = reading.temperature_celsius.to_bits();
                if val_bits < *cx.local.min_dc { *cx.local.min_dc = val_bits; }
                if val_bits > *cx.local.max_dc { *cx.local.max_dc = val_bits; }

                if *cx.local.count % 100 == 0 {
                    let range = *cx.local.max_dc - *cx.local.min_dc;
                    defmt::info!("Samples: {} | Temp: {} | Range: {} bits", 
                        *cx.local.count, reading.temperature_celsius, range);
                    
                    // Reset stats for next window
                    *cx.local.min_dc = i32::MAX;
                    *cx.local.max_dc = i32::MIN;
                }
            }
        }
    }
}
