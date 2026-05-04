#![no_std]
#![no_main]

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [EXTI1])]
mod app {
    use defmt::info;
    use defmt_rtt as _;
    use panic_probe as _;

    use smt160_driver::decoder::Smt160Decoder;
    use stm32f1xx_hal::{pac::TIM2, prelude::*};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        decoder: Smt160Decoder,
        tim2: TIM2,
        timer_overflows: u32,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let dp = cx.device;
        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();

        // Setup the clock tree to 72 MHz
        let _clocks = rcc.cfgr.sysclk(72.MHz()).freeze(&mut flash.acr);

        // Configure PA0 as a floating input (connected to TIM2 Channel 1)
        let mut gpioa = dp.GPIOA.split();
        let _pa0 = gpioa.pa0.into_floating_input(&mut gpioa.crl);

        // --- Hardware TIM2 Configuration ---
        let tim2 = dp.TIM2;

        // Enable TIM2 clock via APB1
        let rcc_pac = unsafe { &*stm32f1xx_hal::pac::RCC::ptr() };
        rcc_pac.apb1enr.modify(|_, w| w.tim2en().set_bit());

        // Set Prescaler for 1us ticks (72MHz / 72 = 1MHz timer clock)
        tim2.psc.write(|w| w.psc().bits(71));

        // Auto-reload register to max for a 16-bit timer
        tim2.arr.write(|w| w.arr().bits(0xFFFF));

        // Smart Capture: Route TI1 (PA0) to both IC1 and IC2
        // This allows capturing both edges simultaneously without race conditions
        tim2.ccmr1_input().write(|w| unsafe {
            w.cc1s()
                .bits(0b01) // IC1 mapped to TI1
                .cc2s()
                .bits(0b10) // IC2 mapped to TI1
        });

        // Enable Captures and Set Polarities
        tim2.ccer.modify(|_, w| {
            w.cc1e()
                .set_bit() // Enable Capture 1
                .cc1p()
                .clear_bit() // Rising Edge trigger
                .cc2e()
                .set_bit() // Enable Capture 2
                .cc2p()
                .set_bit() // Falling Edge trigger
        });

        // Enable Overflow (UIE) and Capture interrupts (CC1IE, CC2IE)
        tim2.dier
            .write(|w| w.uie().set_bit().cc1ie().set_bit().cc2ie().set_bit());

        // Start the Timer
        tim2.cr1.modify(|_, w| w.cen().set_bit());

        (
            Shared {},
            Local {
                decoder: Smt160Decoder::new(),
                tim2,
                timer_overflows: 0,
            },
        )
    }

    // Phase 3: Hardware Accuracy via Interrupt-driven capture
    #[task(binds = TIM2, local = [decoder, tim2, timer_overflows])]
    fn tim2_isr(cx: tim2_isr::Context) {
        let tim2 = cx.local.tim2;
        let overflows = cx.local.timer_overflows;
        let decoder = cx.local.decoder;

        let sr = tim2.sr.read();

        // 1. Handle Timer Overflow (creating a virtual 48-bit timer from the 16-bit one)
        if sr.uif().bit_is_set() {
            *overflows = overflows.wrapping_add(1);
            // Clear the update interrupt flag
            tim2.sr.modify(|_, w| w.uif().clear_bit());
        }

        // 2. Handle Rising Edge (IC1)
        if sr.cc1if().bit_is_set() {
            let capture = tim2.ccr1.read().bits();
            let timestamp_us = ((*overflows as u64) << 16) | (capture as u64);

            match decoder.push_edge(true, timestamp_us) {
                Ok(Some(temp)) => info!("Temperature: {} C", temp.to_num::<f32>()),
                Ok(None) => {} // Engine needs more pulses to stabilize
                Err(e) => defmt::error!("Sensor Error: {:?}", e),
            }

            // Clear the capture 1 flag
            tim2.sr.modify(|_, w| w.cc1if().clear_bit());
        }

        // 3. Handle Falling Edge (IC2)
        if sr.cc2if().bit_is_set() {
            let capture = tim2.ccr2.read().bits();
            let timestamp_us = ((*overflows as u64) << 16) | (capture as u64);

            match decoder.push_edge(false, timestamp_us) {
                Ok(Some(temp)) => info!("Temperature: {} C", temp.to_num::<f32>()),
                Ok(None) => {}
                Err(e) => defmt::error!("Sensor Error: {:?}", e),
            }

            // Clear the capture 2 flag
            tim2.sr.modify(|_, w| w.cc2if().clear_bit());
        }
    }
}
