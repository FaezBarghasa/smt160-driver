#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;
use rtic::app;
use smt160_driver::{Smt160Driver, Ready, Config};
use smt160_driver::hal::stm32f1_dma::Stm32F1DmaHal;
use stm32f1xx_hal::{pac, prelude::*};
use fixed::types::I32F32;

#[app(device = pac, dispatchers = [SPI1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        driver: Smt160Driver<Stm32F1DmaHal<pac::TIM2, stm32f1xx_hal::dma::dma1::C4>, Ready>,
    }

    #[local]
    struct Local {
        samples: &'static mut [I32F32; 1000],
        current_idx: usize,
    }

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

        // PA0 is TIM2_CH1 (TI1)
        let mut gpioa = cx.device.GPIOA.split(&mut rcc);
        let _pin = gpioa.pa0.into_floating_input(&mut gpioa.crl);

        static mut DMA_BUFFER: [u32; 4] = [0; 4];
        static mut SAMPLES: [I32F32; 1000] = [I32F32::ZERO; 1000];

        let dma1 = cx.device.DMA1.split(&mut rcc);
        
        // TIM2_CH1 DMA request is on Channel 4
        let hal = Stm32F1DmaHal::new(cx.device.TIM2, dma1.4, unsafe {
            &mut *core::ptr::addr_of_mut!(DMA_BUFFER)
        });
        let driver = Smt160Driver::new(hal, Config::industrial())
            .init(72_000_000)
            .unwrap();

        (Shared { driver }, Local {
            samples: unsafe { &mut *core::ptr::addr_of_mut!(SAMPLES) },
            current_idx: 0,
        })
    }

    #[task(binds = DMA1_CHANNEL4, shared = [driver], local = [samples, current_idx])]
    fn on_dma(mut cx: on_dma::Context) {
        let samples = cx.local.samples;
        let idx = cx.local.current_idx;

        cx.shared.driver.lock(|driver| {
            if let Some(temp) = driver.read_temperature() {
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
