//! # SMT160 Industrial App: Production Golden Example
//!
//! This example demonstrates a production-grade implementation of the SMT160 driver 
//! on an STM32F103 (BluePill) using RTIC 2.1. 
//!
//! ## Key Features
//! - **Managed Hardware**: Driver owns TIM2 and DMA1 Channel 7.
//! - **Zero CPU Polling**: DMA captures pulse data in the background.
//! - **Task Prioritization**: Sensor processing at Priority 1, UI at Priority 2.
//! - **Performance**: Target <1% CPU usage at 72MHz.

#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [USART1])]
mod app {
    use smt160_driver::platform::stm32f1_managed::{Smt160Dma, DMA_BUFFER_SIZE};
    use smt160_driver::decoder::Smt160Decoder;
    use smt160_driver::config::{StaticConfiguration, Smt160Config};
    use stm32f1xx_hal::prelude::*;

    #[shared]
    struct Shared {
        last_temperature: Option<smt160_driver::Reading>,
    }

    // Static buffer for DMA capture (must be 'static)
    #[unsafe(link_section = ".sram")]
    static mut DMA_BUFFER: [u32; DMA_BUFFER_SIZE] = [0; DMA_BUFFER_SIZE];

    #[local]
    struct Local {
        sensor: Smt160Dma,
        decoder: Smt160Decoder,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();
        let mut rcc = rcc.freeze(
            stm32f1xx_hal::rcc::Config::hse(8.MHz())
                .sysclk(72.MHz())
                .pclk1(36.MHz()),
            &mut flash.acr,
        );

        let mut gpioa = cx.device.GPIOA.split(&mut rcc); // TIM2 CH2
        let _smt160_pin = gpioa.pa1.into_floating_input(&mut gpioa.crl); // TIM2 CH2

        // Setup DMA1
        let dma1 = cx.device.DMA1.split(&mut rcc);
        
        // One-line setup as requested in production goals
        // Note: Using unsafe for the static buffer in init
        let buffer_ref = unsafe { &mut *core::ptr::addr_of_mut!(DMA_BUFFER) };
        let sensor = Smt160Dma::new(cx.device.TIM2, dma1.7, 72, buffer_ref).unwrap();
        
        let decoder = Smt160Decoder::new_standalone(72);

        defmt::info!("SMT160 Industrial App Initialized");

        (
            Shared { last_temperature: None },
            Local { sensor, decoder },
        )
    }

    /// High-Priority Sensor Task (Priority 1)
    /// Processes DMA buffer completions.
    #[task(binds = DMA1_CHANNEL7, local = [sensor, decoder], shared = [last_temperature], priority = 1)]
    fn sensor_isr(mut cx: sensor_isr::Context) {
        let sensor = cx.local.sensor;
        let decoder = cx.local.decoder;

        if sensor.poll_status() {
            sensor.clear_flags();
            
            // Industrial Efficiency: Process batch from DMA circular buffer
            let buffer = sensor.get_capture_buffer();
            let (offset, step) = StaticConfiguration.get_offsets();
            
            match decoder.process_batch(buffer, offset, step) {
                Ok(Some(reading)) => {
                    cx.shared.last_temperature.lock(|temp| *temp = Some(reading));
                    defmt::trace!("Temp: {} C", reading.temperature_celsius.to_num::<f32>());
                }
                _ => {}
            }
        }
    }

    /// Low-Priority UI Task (Priority 2)
    /// Demonstrates coexistence and multi-tasking.
    #[task(shared = [last_temperature], priority = 2)]
    async fn ui_task(mut cx: ui_task::Context) {
        loop {
            // UI refresh logic here
            cx.shared.last_temperature.lock(|temp| {
                if let Some(t) = temp {
                    defmt::info!("Display Update: {} C", t.temperature_celsius.to_num::<f32>());
                }
            });
            // Systick or other monotonic sleep
        }
    }
}
