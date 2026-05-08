#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;
use rtic::app;
use smt160_driver::{Smt160, Ready};
use stm32f1xx_hal::{pac, prelude::*};
use fixed::types::I32F32;

#[app(device = pac, dispatchers = [SPI1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        driver: Smt160<Ready, pac::TIM2, stm32f1xx_hal::dma::dma1::C5>,
    }

    #[local]
    struct Local {
        samples: &'static mut [I32F32; 1000],
        current_idx: usize,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.freeze(
            stm32f1xx_hal::rcc::Config::hse(8.MHz())
                .sysclk(72.MHz())
                .pclk1(36.MHz()),
            &mut flash.acr,
        );

        let clocks = rcc.clocks;

        let mut gpioa = cx.device.GPIOA.split(&mut rcc);
        let _pin = gpioa.pa1.into_floating_input(&mut gpioa.crl);

        static mut DMA_BUFFER: [u32; 4] = [0; 4];
        static mut SAMPLES: [I32F32; 1000] = [I32F32::ZERO; 1000];

        let dma1 = cx.device.DMA1.split(&mut rcc);
        let driver = Smt160::new(cx.device.TIM2, dma1.5, unsafe { &mut DMA_BUFFER })
            .init(&clocks)
            .unwrap();

        (Shared { driver }, Local { samples: unsafe { &mut SAMPLES }, current_idx: 0 })
    }

    #[task(binds = DMA1_CHANNEL5, shared = [driver], local = [samples, current_idx])]
    fn on_dma(mut cx: on_dma::Context) {
        let samples = cx.local.samples;
        let idx = cx.local.current_idx;

        cx.shared.driver.lock(|driver| {
            if let Some(temp) = driver.poll_dma() {
                if *idx < samples.len() {
                    samples[*idx] = temp;
                    *idx += 1;
                } else {
                    // Benchmark Complete: Calculate stats
                    let stats = calculate_stats(&samples[..]);
                    defmt::info!("Jitter Benchmark Complete (1000 samples)");
                    defmt::info!("Mean: {} °C", stats.mean.to_num::<f32>());
                    defmt::info!("StdDev: {} °C", stats.std_dev.to_num::<f32>());
                    defmt::info!("Jitter Spread: {} °C", stats.spread.to_num::<f32>());
                    
                    if stats.std_dev < I32F32::from_num(0.05) {
                        defmt::info!("RESULT: Precision Target MET (<0.05°C)");
                    } else {
                        defmt::error!("RESULT: Precision Target FAILED");
                    }
                    
                    *idx = 0; // Restart
                }
            }
        });
    }

    struct Stats {
        mean: I32F32,
        std_dev: I32F32,
        spread: I32F32,
    }

    fn calculate_stats(data: &[I32F32]) -> Stats {
        let mut sum = I32F32::ZERO;
        let mut min = data[0];
        let mut max = data[0];
        
        for &val in data {
            sum += val;
            if val < min { min = val; }
            if val > max { max = val; }
        }
        
        let mean = sum / I32F32::from_num(data.len());
        
        let mut variance_sum = I32F32::ZERO;
        for &val in data {
            let diff = val - mean;
            variance_sum += diff * diff;
        }
        
        let variance = variance_sum / I32F32::from_num(data.len());
        
        // Fixed-point square root using libm or simple approximation
        let std_dev = I32F32::from_num(libm::sqrt(variance.to_num::<f64>()));
        
        Stats {
            mean,
            std_dev,
            spread: max - min,
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop { cortex_m::asm::wfi(); }
    }
}
