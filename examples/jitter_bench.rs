#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;
use rtic::app;
use smt160_driver::{Smt160Dma, Ready};
use stm32f1xx_hal::{pac, prelude::*};
use fixed::types::I32F32;

#[app(device = pac, dispatchers = [SPI1])]
mod app {
    use super::*;

    const SAMPLE_COUNT: usize = 10_000;

    #[shared]
    struct Shared {
        driver: Smt160Dma<Ready, pac::TIM2, pac::DMA1_CH5>,
    }

    #[local]
    struct Local {
        samples: &'static mut [I32F32; SAMPLE_COUNT],
        current_idx: usize,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        let clocks = rcc.cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .pclk1(36.MHz())
            .freeze(&mut flash.acr);

        let mut gpioa = cx.device.GPIOA.split();
        let _pin = gpioa.pa1.into_floating_input(&mut gpioa.crl);

        static mut DMA_BUFFER: [u32; 4] = [0; 4];
        static mut SAMPLES: [I32F32; SAMPLE_COUNT] = [I32F32::ZERO; SAMPLE_COUNT];

        let rcc_pac = unsafe { &*pac::RCC::ptr() };
        rcc_pac.apb1enr.modify(|_, w| w.tim2en().set_bit());
        rcc_pac.ahbenr.modify(|_, w| w.dma1en().set_bit());

        let driver = Smt160Dma::new(cx.device.TIM2, cx.device.DMA1_CH5, unsafe { &mut DMA_BUFFER })
            .init(&clocks)
            .unwrap();

        defmt::info!("Jitter Benchmark Started. Collecting {} samples...", SAMPLE_COUNT);

        (Shared { driver }, Local { samples: unsafe { &mut SAMPLES }, current_idx: 0 })
    }

    #[task(binds = DMA1_CHANNEL5, shared = [driver], local = [samples, current_idx])]
    fn on_dma(mut cx: on_dma::Context) {
        let idx = *cx.local.current_idx;
        if idx >= SAMPLE_COUNT {
            return;
        }

        cx.shared.driver.lock(|drv| {
            if let Some(temp) = drv.poll_dma() {
                cx.local.samples[idx] = temp;
                *cx.local.current_idx += 1;

                if *cx.local.current_idx == SAMPLE_COUNT {
                    defmt::info!("Collection complete. Calculating statistics...");
                    calculate_statistics(cx.local.samples);
                }
            }
        });
    }

    fn calculate_statistics(samples: &[I32F32]) {
        let mut sum = 0.0f64;
        for &s in samples {
            sum += s.to_num::<f64>();
        }
        let mean = sum / (samples.len() as f64);

        let mut variance_sum = 0.0f64;
        for &s in samples {
            let diff = s.to_num::<f64>() - mean;
            variance_sum += diff * diff;
        }
        let variance = variance_sum / (samples.len() as f64);
        let std_dev = libm::sqrt(variance);

        defmt::info!("--- Benchmark Results ---");
        defmt::info!("Samples: {}", samples.len());
        defmt::info!("Mean: {} °C", mean as f32);
        defmt::info!("Std Dev: {} °C", std_dev as f32);
        defmt::info!("Peak-to-Peak Jitter: {} °C", (std_dev * 6.0) as f32); // 6-sigma
        
        if std_dev < 0.05 {
            defmt::info!("SUCCESS: Standard deviation is < 0.05°C. Claim verified.");
        } else {
            defmt::warn!("FAILURE: Standard deviation is {}°C. Check hardware connection.", std_dev as f32);
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}
