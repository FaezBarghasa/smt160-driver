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

        validate_clocks(&clocks).expect("Clock validation failed: APB1 must be >= 8MHz");

        defmt::info!("SMT160 Production Driver Initializing...");
        defmt::info!("System Clock: 72000000 Hz");

        // PA0 is TIM2_CH1 (TI1) for the SMT160 Signal Input
        let mut gpioa = cx.device.GPIOA.split();
        let _pin = gpioa.pa0.into_pull_up_input(&mut gpioa.crl);
        defmt::info!("GPIO Initialized (Pull-Up)");

        // Circular DMA Buffer for CCR1 and CCR2 captures
        static mut BUF: smt160_driver::hal::stm32f1_dma::Smt160DmaBuffer<100> = 
            smt160_driver::hal::stm32f1_dma::Smt160DmaBuffer::new();

        let channels = cx.device.DMA1.split();
        defmt::info!("DMA Initialized");

        // TIM2_CH1 DMA request is on Channel 5
        let hal = Stm32F1DmaHal::new(cx.device.TIM2, channels.5, unsafe {
            &mut *core::ptr::addr_of_mut!(BUF)
        }, 1, 100);

        let driver = Smt160Driver::new(hal, Config::industrial(), Mono::now())
            .init(72_000_000)
            .expect("Driver initialization failed");
        defmt::info!("Driver Initialized");

        // Initialize Systick for 1ms resolution watchdog (72MHz)
        Mono::start(cx.core.SYST, 72_000_000);
        defmt::info!("Monotonic Started");

        watchdog::spawn().ok();

        defmt::info!("Returning from init");
        (Shared { driver }, Local {})
    }

    /// High-Priority DMA Task: Triggered on Half-Transfer or Transfer-Complete
    #[task(binds = DMA1_CHANNEL5, shared = [driver], priority = 2)]
    fn on_dma(mut cx: on_dma::Context) {
        defmt::info!("DMA Interrupt Fired");
        cx.shared.driver.lock(|driver| {
            driver.hal_mut().notify();
            if let Some(temp) = driver.read_temperature::<Mono>() {
                defmt::info!("Temperature: {} °C", temp.to_num::<f32>());
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
            
            // Diagnostic: Read raw hardware state
            let _tim2_cnt = unsafe { (*pac::TIM2::ptr()).cnt.read().bits() };
            let _dma1_isr = unsafe { (*pac::DMA1::ptr()).isr.read().bits() };
            let _tim2_dier = unsafe { (*pac::TIM2::ptr()).dier.read().bits() };
            let _tim2_ccer = unsafe { (*pac::TIM2::ptr()).ccer.read().bits() };
            let _tim2_sr = unsafe { (*pac::TIM2::ptr()).sr.read().bits() };
            let _tim2_dcr = unsafe { (*pac::TIM2::ptr()).dcr.read().bits() };
            let dma1_ptr = pac::DMA1::ptr() as u32;
            let dma1_ccr5_raw = unsafe { core::ptr::read_volatile(0x40020058 as *const u32) };
            let dma1_cndtr5_raw = unsafe { core::ptr::read_volatile(0x4002005C as *const u32) };
            let dma1_isr_raw = unsafe { core::ptr::read_volatile(0x40020000 as *const u32) };
            let rcc_ahbenr = unsafe { (*pac::RCC::ptr()).ahbenr.read().bits() };

            defmt::info!("Watchdog Tick | Base: {:#X} | ISR: {:#X} | CNDTR: {} | CCR: {:#X} | AHB: {:#X}", dma1_ptr, dma1_isr_raw, dma1_cndtr5_raw, dma1_ccr5_raw, rcc_ahbenr);
 
            cx.shared.driver.lock(|driver| {
                if let Some(temp) = driver.read_temperature::<Mono>() {
                    defmt::info!("Watchdog Backup Read: {} °C", temp.to_num::<f32>());
                }

                if driver.status().contains(Smt160Status::SENSOR_TIMEOUT) {
                    defmt::info!(
                        "Sensor Flatline Detected! Attempting autonomous hardware recovery..."
                    );
                    let _ = driver.reinit(72_000_000, Mono::now());
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
