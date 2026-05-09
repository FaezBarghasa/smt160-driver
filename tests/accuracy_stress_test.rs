#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;
use rtic::app;
use rtic_monotonics::systick_monotonic;

systick_monotonic!(Mono, 1_000);

use smt160_driver::hal::stm32f1_dma::{Stm32F1DmaHal, validate_clocks};
use smt160_driver::{Config, Ready, Smt160Driver, Smt160Status};
use stm32f1xx_hal::{pac, prelude::*};

#[app(device = pac, dispatchers = [USART1])]
mod app {
    use super::*;
    use rtic_monotonics::Monotonic;

    #[shared]
    struct Shared {
        driver: Smt160Driver<Stm32F1DmaHal<pac::TIM2, stm32f1xx_hal::dma::dma1::C4>, Ready>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut flash = cx.device.FLASH.constrain();
        cx.device.RCC.apb1enr().modify(|_, w| w.tim2en().set_bit());

        let mut rcc = cx.device.RCC.freeze(
            stm32f1xx_hal::rcc::Config::hse(8.MHz())
                .sysclk(72.MHz())
                .pclk1(36.MHz()),
            &mut flash.acr,
        );

        let clocks = rcc.clocks;
        validate_clocks(&clocks).expect("Clock validation failed");

        static mut DMA_BUFFER: smt160_driver::hal::stm32f1_dma::Smt160DmaBuffer = 
            smt160_driver::hal::stm32f1_dma::Smt160DmaBuffer::new();

        let dma1 = cx.device.DMA1.split(&mut rcc);
        let hal = Stm32F1DmaHal::new(cx.device.TIM2, dma1.4, unsafe {
            &mut *core::ptr::addr_of_mut!(DMA_BUFFER)
        }, 1);

        let driver = Smt160Driver::new(hal, Config::industrial())
            .init(72_000_000)
            .unwrap();

        Mono::start(cx.core.SYST, 72_000_000);

        stress_test::spawn().ok();

        (Shared { driver }, Local {})
    }

    #[task(shared = [driver], priority = 1)]
    async fn stress_test(mut cx: stress_test::Context) {
        defmt::info!("Starting Accuracy Stress Test...");
        let mut samples = 0;
        
        loop {
            // Wait for update asynchronously
            let wait_res = cx.shared.driver.lock(|driver| async {
                driver.wait_for_update().await
            }).await;

            if wait_res.is_ok() {
                samples += 1;
                let temp = cx.shared.driver.lock(|driver| {
                    driver.read_temperature::<Mono>()
                });

                if let Some(t) = temp {
                    if samples % 100 == 0 {
                        let (std_dev, status) = cx.shared.driver.lock(|driver| {
                            (driver.diagnostics.std_dev(), driver.status())
                        });
                        
                        defmt::info!("Sample {}: {} °C (StdDev: {})", samples, t.to_num::<f32>(), std_dev);
                        
                        // Assertions for HIL
                        if status.contains(Smt160Status::SENSOR_TIMEOUT) {
                            defmt::error!("Test Failed: Sensor Timeout");
                            panic!("HIL Test Failed");
                        }
                    }
                }
            }

            if samples >= 1000 {
                defmt::info!("Stress Test Passed: 1000 samples collected successfully.");
                cortex_m::asm::bkpt();
                break;
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
