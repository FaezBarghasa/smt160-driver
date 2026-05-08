//! Production RTIC 2.1 Template for SMT160 Industrial Deployment.
//!
//! This example demonstrates how to configure the STM32F103 clocks, 
//! initialize the SMT160 driver with DMA Burst mode, and process 
//! readings in a high-priority hardware task.

#![no_main]
#![no_std]

use defmt_rtt as _; // global logger
use panic_probe as _;

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [EXTI0])]
mod app {
    use smt160_driver::{Smt160Dma, Uninitialized, Ready};
    use stm32f1xx_hal::{
        pac,
        prelude::*,
        gpio::{gpioa::PA1, Input, Floating},
        timer::Timer,
    };
    use fixed::types::I32F32;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        smt160: Smt160Dma<Ready, pac::TIM2, pac::DMA1_CH5>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        // 1. Configure Clocks to exactly 72MHz for maximum precision
        let clocks = rcc.cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .pclk1(36.MHz()) // PCLK1 is at 36MHz, Timers at 72MHz (2x)
            .freeze(&mut flash);

        let mut gpioa = cx.device.GPIOA.split();
        
        // 2. Configure PA1 for TIM2_CH2 (Input Capture)
        let _pin = gpioa.pa1.into_floating_input(&mut gpioa.crl);

        // 3. Static buffer for DMA Double Buffering
        static mut BUFFER: [u32; 4] = [0u32; 4];

        // 4. Initialize SMT160 Driver
        let smt160_uninit = Smt160Dma::new(
            cx.device.TIM2, 
            cx.device.DMA1_CH5, 
            unsafe { &mut *core::ptr::addr_of_mut!(BUFFER) }
        );

        let smt160 = smt160_uninit.init(&clocks).expect("Clock validation failed");

        defmt::info!("SMT160 Industrial Driver Initialized at 72MHz");

        (Shared {}, Local { smt160 })
    }

    /// High-Priority Hardware Task triggered by DMA Transfer Complete/Half-Transfer
    #[task(binds = DMA1_CHANNEL5, local = [smt160], priority = 5)]
    fn on_dma_capture(cx: on_dma_capture::Context) {
        // Poll the driver for new temperature data
        if let Some(temp) = cx.local.smt160.poll_dma() {
            defmt::info!("Temperature: {} °C", temp.to_num::<f32>());
        }

        // Handle jitter or timeout alerts
        if cx.local.smt160.status.contains(smt160_driver::Smt160Status::JITTER_DETECTED) {
            defmt::warn!("Industrial Alert: High EMI/Jitter Detected");
        }

        // Clear DMA flags to acknowledge the interrupt
        // In a real implementation, you'd clear the IFCR register here.
    }

    /// Background monitoring task
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            core::hint::spin_loop();
        }
    }
}
