#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;
use rtic::app;
use rtic_monotonics::systick_monotonic;

systick_monotonic!(Mono, 1_000);

use rtic_monotonics::Monotonic;
use smt160_driver::hal::stm32f1_dma::{Stm32F1DmaHal, validate_clocks};
use smt160_driver::{Config, Ready, Smt160Driver, Smt160Status};
use stm32f1xx_hal::{pac, prelude::*};

#[app(device = pac, dispatchers = [USART1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        driver: Smt160Driver<Stm32F1DmaHal<pac::TIM2, stm32f1xx_hal::dma::dma1::C4>, Ready>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut flash = cx.device.FLASH.constrain();
        // CRITICAL: Enable TIM2 peripheral clock before access
        cx.device.RCC.apb1enr().modify(|_, w| w.tim2en().set_bit());

        let mut rcc = cx.device.RCC.freeze(
            stm32f1xx_hal::rcc::Config::hse(8.MHz())
                .sysclk(72.MHz())
                .pclk1(36.MHz()),
            &mut flash.acr,
        );

        let clocks = rcc.clocks;
        validate_clocks(&clocks).expect("Clock validation failed: APB1 must be >= 8MHz");

        defmt::info!("SMT160 Production Driver Initializing...");
        defmt::info!("System Clock: 72000000 Hz");

        // PA0 is TIM2_CH1 (TI1) for the SMT160 Signal Input
        let mut gpioa = cx.device.GPIOA.split(&mut rcc);
        let _pin = gpioa.pa0.into_floating_input(&mut gpioa.crl);
        defmt::info!("GPIO Initialized");

        // Circular DMA Buffer for CCR1 and CCR2 captures
        // Format: [CCR1_0, CCR2_0, CCR1_1, CCR2_1]
        static mut DMA_BUFFER: [u32; 4] = [0; 4];

        let dma1 = cx.device.DMA1.split(&mut rcc);
        defmt::info!("DMA Initialized");

        // TIM2_CH1 DMA request is on Channel 4
        let hal = Stm32F1DmaHal::new(cx.device.TIM2, dma1.4, unsafe {
            &mut *core::ptr::addr_of_mut!(DMA_BUFFER)
        });

        let driver = Smt160Driver::new(hal, Config::industrial())
            .init(72_000_000)
            .unwrap();
        defmt::info!("Driver Initialized");

        // Initialize Systick for 1ms resolution watchdog (72MHz)
        Mono::start(cx.core.SYST, 72_000_000);
        defmt::info!("Monotonic Started");

        watchdog::spawn().ok();

        defmt::info!("Returning from init");
        (Shared { driver }, Local {})
    }

    /// High-Priority DMA Task: Triggered on Half-Transfer or Transfer-Complete
    #[task(binds = DMA1_CHANNEL4, shared = [driver], priority = 2)]
    fn on_dma(mut cx: on_dma::Context) {
        defmt::info!("DMA Interrupt Fired");
        cx.shared.driver.lock(|driver| {
            if let Some(temp) = driver.read_temperature() {
                defmt::info!("Temperature: {} °C", temp.to_num::<f32>());

                let status = driver.status();
                if status.contains(Smt160Status::JITTER_DETECTED) {
                    defmt::warn!("Signal Jitter Detected! Check EMI/Wiring.");
                }
            }
        });
    }

    /// Background Watchdog Task: Monitors sensor health and performs auto-recovery
    #[task(shared = [driver], priority = 1)]
    async fn watchdog(mut cx: watchdog::Context) {
        defmt::info!("Watchdog Task Started");
        loop {
            // Check sensor health every 10ms
            Mono::delay(10.millis()).await;
            defmt::info!("Watchdog Tick");

            cx.shared.driver.lock(|driver| {
                // Diagnostic: Check if data is arriving
                if let Some(temp) = driver.read_temperature() {
                    defmt::info!("Watchdog Backup Read: {} °C", temp.to_num::<f32>());
                }

                if driver.status().contains(Smt160Status::SENSOR_TIMEOUT) {
                    defmt::info!(
                        "Sensor Flatline Detected! Attempting autonomous hardware recovery..."
                    );
                    // Re-initialize the HAL to reset hardware state
                    let _ = driver.reinit(72_000_000);
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
