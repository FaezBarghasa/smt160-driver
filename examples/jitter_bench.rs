//! Jitter Benchmark Tool for SMT160.
//!
//! Accumulates 10,000 samples and computes the standard deviation 
//! to mathematically prove the <0.05°C precision claim.

#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [EXTI0])]
mod app {
    use smt160_driver::{Smt160Dma, Ready};
    use stm32f1xx_hal::{pac, prelude::*};

    const SAMPLE_COUNT: usize = 10_000;
    static mut SAMPLES: [f32; SAMPLE_COUNT] = [0.0; SAMPLE_COUNT];

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        smt160: Smt160Dma<Ready, pac::TIM2, pac::DMA1_CH5>,
        sample_idx: usize,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(72.MHz()).freeze(&mut flash);

        let mut gpioa = cx.device.GPIOA.split();
        let _pin = gpioa.pa1.into_floating_input(&mut gpioa.crl);

        static mut BUFFER: [u32; 4] = [0u32; 4];
        let smt160 = Smt160Dma::new(
            cx.device.TIM2, 
            cx.device.DMA1_CH5, 
            unsafe { &mut *core::ptr::addr_of_mut!(BUFFER) }
        ).init(&clocks).unwrap();

        defmt::info!("Starting Jitter Benchmark (10,000 samples)...");

        (Shared {}, Local { smt160, sample_idx: 0 })
    }

    #[task(binds = DMA1_CHANNEL5, local = [smt160, sample_idx], priority = 5)]
    fn on_dma(cx: on_dma::Context) {
        if *cx.local.sample_idx < SAMPLE_COUNT {
            if let Some(temp) = cx.local.smt160.poll_dma() {
                let val = temp.to_num::<f32>();
                unsafe { SAMPLES[*cx.local.sample_idx] = val; }
                *cx.local.sample_idx += 1;

                if *cx.local.sample_idx == SAMPLE_COUNT {
                    compute_bench();
                }
            }
        }
    }

    fn compute_bench() {
        let mut sum = 0.0;
        let mut sum_sq = 0.0;
        
        unsafe {
            for &s in SAMPLES.iter() {
                sum += s;
                sum_sq += s * s;
            }
        }

        let mean = sum / SAMPLE_COUNT as f32;
        let variance = (sum_sq / SAMPLE_COUNT as f32) - (mean * mean);
        let std_dev = sqrt_f32(variance);

        defmt::info!("Benchmark Complete!");
        defmt::info!("Mean Temp: {} °C", mean);
        defmt::info!("Standard Deviation: {} °C", std_dev);
        defmt::info!("Peak-to-Peak Jitter: {} °C", std_dev * 6.0); // 6-sigma
        
        if std_dev < 0.05 {
            defmt::info!("PRECISION CLAIM VERIFIED: <0.05°C");
        } else {
            defmt::warn!("Precision below target. Check hardware noise.");
        }
    }

    /// Simple Newton-Raphson sqrt for benchmark (not for production)
    fn sqrt_f32(val: f32) -> f32 {
        let mut x = val;
        let mut y = 1.0;
        let e = 0.00001;
        while (x - y).abs() > e {
            x = (x + y) / 2.0;
            y = val / x;
        }
        x
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop { core::hint::spin_loop(); }
    }
}
