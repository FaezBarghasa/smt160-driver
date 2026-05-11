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
        let _pin = gpioa.pa0.into_floating_input(&mut gpioa.crl);
        defmt::info!("GPIO Initialized (Floating Input)");

        unsafe {
            // Full RCC reset of TIM2 peripheral to guarantee clean register state
            (*pac::RCC::ptr()).apb1rstr.modify(|_, w| w.tim2rst().set_bit());
            (*pac::RCC::ptr()).apb1rstr.modify(|_, w| w.tim2rst().clear_bit());

            // Enable TIM2 and AFIO clocks
            (*pac::RCC::ptr()).apb1enr.modify(|_, w| w.tim2en().set_bit());
            (*pac::RCC::ptr()).apb2enr.modify(|_, w| w.afioen().set_bit());
            let _ = (*pac::RCC::ptr()).apb1enr.read(); // bus sync
        }

        // Circular DMA Buffer for CCR1 and CCR2 captures
        static mut BUF: smt160_driver::hal::stm32f1_dma::Smt160DmaBuffer<100> = 
            smt160_driver::hal::stm32f1_dma::Smt160DmaBuffer::new();

        let channels = cx.device.DMA1.split();
        defmt::info!("DMA Initialized");

        // TIM2_CH1 DMA request is on DMA1 Channel 5
        let hal = Stm32F1DmaHal::new(cx.device.TIM2, channels.5, unsafe {
            &mut *core::ptr::addr_of_mut!(BUF)
        }, 1, 100);

        let timer_freq = 1_000_000; // 1 MHz capture resolution
        let driver = Smt160Driver::new(hal, Config::industrial(), Mono::now())
            .init(timer_freq)
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
            Mono::delay(100.millis()).await;
            
            // Read raw hardware state for diagnostics
            let tim2_cnt = unsafe { (*pac::TIM2::ptr()).cnt.read().bits() };
            let tim2_sr = unsafe { (*pac::TIM2::ptr()).sr.read().bits() };
            let tim2_ccr1 = unsafe { (*pac::TIM2::ptr()).ccr1().read().bits() };
            let tim2_ccr2 = unsafe { (*pac::TIM2::ptr()).ccr2().read().bits() };
            let tim2_ccer = unsafe { (*pac::TIM2::ptr()).ccer.read().bits() };
            let tim2_dier = unsafe { (*pac::TIM2::ptr()).dier.read().bits() };
            let tim2_psc = unsafe { (*pac::TIM2::ptr()).psc.read().bits() };
            let tim2_arr = unsafe { (*pac::TIM2::ptr()).arr.read().bits() };
            let dma_cndtr = unsafe { core::ptr::read_volatile(0x4002005C as *const u32) };
            let dma_isr = unsafe { core::ptr::read_volatile(0x40020000 as *const u32) };

            defmt::info!(
                "CNDTR:{} CNT:{} SR:{:#X} CCR1:{} CCR2:{} CCER:{:#X} DIER:{:#X} PSC:{} ARR:{} DMA_ISR:{:#X}",
                dma_cndtr, tim2_cnt, tim2_sr, tim2_ccr1, tim2_ccr2, tim2_ccer, tim2_dier, tim2_psc, tim2_arr, dma_isr
            );
 
            cx.shared.driver.lock(|driver| {
                if let Some(temp) = driver.read_temperature::<Mono>() {
                    defmt::info!("Watchdog Temp: {} °C", temp.to_num::<f32>());
                }

                if driver.status().contains(Smt160Status::SENSOR_TIMEOUT) {
                    defmt::info!(
                        "Sensor Flatline Detected! Attempting autonomous hardware recovery..."
                    );
                    let _ = driver.reinit(1_000_000, Mono::now());
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
