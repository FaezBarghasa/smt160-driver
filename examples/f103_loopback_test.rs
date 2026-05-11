//! # SMT160 Loopback Self-Test
//!
//! This example generates a known PWM signal on TIM3/PA6 that simulates an
//! SMT160 sensor output, then captures it on TIM2/PA0 via the full DMA pipeline.
//!
//! ## Wiring Required:
//!   Connect PA6 → PA0 with a single jumper wire.
//!
//! ## Expected Output:
//!   The driver should report temperature readings corresponding to the
//!   simulated duty cycle (~50% = ~25°C).
//!
//! ```
//! cargo run --release --example f103_loopback_test
//! ```

#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;
use rtic::app;
use rtic_monotonics::systick_monotonic;

systick_monotonic!(Mono, 1_000);

#[app(device = pac, dispatchers = [USART1])]
mod app {
    use stm32f1xx_hal::{pac, prelude::*};
    use smt160_driver::hal::stm32f1_dma::{Stm32F1DmaHal, validate_clocks};
    use smt160_driver::hal::Smt160Hal;
    use smt160_driver::{Config, Ready, Smt160Driver, Smt160Status};
    use rtic_monotonics::Monotonic;
    use super::Mono;

    #[shared]
    struct Shared {
        driver: Smt160Driver<Stm32F1DmaHal<'static, pac::TIM2, stm32f1xx_hal::dma::dma1::C5, 100>, Ready>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        let clocks = rcc.cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .pclk1(36.MHz())
            .freeze(&mut flash.acr);

        validate_clocks(&clocks).expect("Clock validation failed");

        defmt::info!("=== SMT160 Loopback Self-Test ===");
        defmt::info!("Wire PA6 -> PA0 to run this test");

        let mut gpioa = cx.device.GPIOA.split();

        // PA0: TIM2_CH1 input capture (floating input for timer capture)
        let _capture_pin = gpioa.pa0.into_floating_input(&mut gpioa.crl);

        // PA6: TIM3_CH1 PWM output (alternate function push-pull)
        let pwm_pin = gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl);

        // ---- Generate simulated SMT160 signal on TIM3/PA6 ----
        // SMT160 outputs ~1kHz-4kHz PWM. Duty cycle encodes temperature:
        //   duty = 0.320 + 0.00470 * T(°C)
        //   At 25°C: duty = 0.320 + 0.1175 = 0.4375 (43.75%)
        //   At 40°C: duty = 0.320 + 0.188  = 0.508  (50.8%)
        //
        // We'll generate a 1kHz PWM at ~50% duty (≈ ~38.3°C)
        let pwm = cx.device.TIM3.pwm_hz(pwm_pin, &clocks, 1.kHz());
        let mut ch1 = pwm.split();
        let max_duty = ch1.get_max_duty();
        // 50% duty cycle → ~38.3°C according to SMT160 formula
        ch1.set_duty(max_duty / 2);
        ch1.enable();
        defmt::info!("PWM Generator: 1kHz, 50% duty on PA6 (max_duty={})", max_duty);

        // ---- Set up TIM2 DMA capture on PA0 ----
        unsafe {
            // Enable TIM2 and AFIO clocks
            (*pac::RCC::ptr()).apb1enr.modify(|_, w| w.tim2en().set_bit());
            (*pac::RCC::ptr()).apb2enr.modify(|_, w| w.afioen().set_bit());
            let _ = (*pac::RCC::ptr()).apb1enr.read();
        }

        static mut BUF: smt160_driver::hal::stm32f1_dma::Smt160DmaBuffer<100> =
            smt160_driver::hal::stm32f1_dma::Smt160DmaBuffer::new();

        let channels = cx.device.DMA1.split();
        defmt::info!("DMA Initialized");

        let hal = Stm32F1DmaHal::new(cx.device.TIM2, channels.5, unsafe {
            &mut *core::ptr::addr_of_mut!(BUF)
        }, 1, 100);

        let timer_freq = 1_000_000; // 1 MHz capture resolution
        let driver = Smt160Driver::new(hal, Config::industrial(), Mono::now())
            .init(timer_freq)
            .expect("Driver initialization failed");
        defmt::info!("Driver Initialized at {} Hz capture", timer_freq);

        Mono::start(cx.core.SYST, 72_000_000);
        watchdog::spawn().ok();

        defmt::info!("Loopback test running...");
        (Shared { driver }, Local {})
    }

    #[task(binds = DMA1_CHANNEL5, shared = [driver], priority = 2)]
    fn on_dma(mut cx: on_dma::Context) {
        cx.shared.driver.lock(|driver| {
            driver.hal_mut().notify();
            if let Some(temp) = driver.read_temperature::<Mono>() {
                defmt::info!("DMA ISR Temperature: {} °C", temp.to_num::<f32>());
            }
        });
    }

    #[task(shared = [driver], priority = 1)]
    async fn watchdog(mut cx: watchdog::Context) {
        defmt::info!("Watchdog Started");
        loop {
            Mono::delay(100.millis()).await;

            let cndtr = unsafe { core::ptr::read_volatile(0x4002005C as *const u32) };
            let sr = unsafe { (*pac::TIM2::ptr()).sr.read().bits() };
            let cnt = unsafe { (*pac::TIM2::ptr()).cnt.read().bits() };

            cx.shared.driver.lock(|driver| {
                if let Some(temp) = driver.read_temperature::<Mono>() {
                    defmt::info!("Temp: {:.2} °C | CNDTR: {} | SR: {:#X} | CNT: {}", temp.to_num::<f32>(), cndtr, sr, cnt);
                } else {
                    defmt::info!("No data | CNDTR: {} | SR: {:#X} | CNT: {}", cndtr, sr, cnt);
                }

                if driver.status().contains(Smt160Status::SENSOR_TIMEOUT) {
                    defmt::warn!("Sensor timeout - check PA6→PA0 wire!");
                }
            });
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}
