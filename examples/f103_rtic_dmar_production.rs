#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;
use rtic::app;
use smt160_driver::{Smt160Dma, Uninitialized, Ready};
use stm32f1xx_hal::{
    pac,
    prelude::*,
    gpio::{gpioa::PA1, Input, Floating},
};

#[app(device = pac, dispatchers = [SPI1])]
mod app {
    use super::*;
    use smt160_driver::Smt160Status;

    #[shared]
    struct Shared {
        driver: Smt160Dma<Ready, pac::TIM2, pac::DMA1_CH5>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        // 72MHz is the standard high-performance clock for STM32F103
        let clocks = rcc.cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .pclk1(36.MHz())
            .freeze(&mut flash.acr);

        defmt::info!("System Clock: {} Hz", clocks.sysclk().to_Hz());

        let mut gpioa = cx.device.GPIOA.split();
        let _pin = gpioa.pa1.into_floating_input(&mut gpioa.crl);

        // Static buffer for DMA double-buffering
        static mut DMA_BUFFER: [u32; 4] = [0; 4];

        // Enable peripheral clocks manually as required by PAC access
        let rcc_pac = unsafe { &*pac::RCC::ptr() };
        rcc_pac.apb1enr.modify(|_, w| w.tim2en().set_bit());
        rcc_pac.ahbenr.modify(|_, w| w.dma1en().set_bit());

        // Initialize driver
        let driver_uninit = Smt160Dma::new(cx.device.TIM2, cx.device.DMA1_CH5, unsafe { &mut DMA_BUFFER });
        let driver = driver_uninit.init(&clocks).expect("Clock validation failed");

        defmt::info!("SMT160 Driver Initialized");

        (Shared { driver }, Local {})
    }

    #[task(binds = DMA1_CHANNEL5, shared = [driver])]
    fn on_dma(mut cx: on_dma::Context) {
        cx.shared.driver.lock(|drv| {
            if let Some(temp) = drv.poll_dma() {
                let status = drv.status;
                if status.contains(Smt160Status::JITTER_DETECTED) {
                    defmt::warn!("Jitter detected! Signal integrity compromised.");
                }
                defmt::info!("Temperature: {} °C", temp.to_num::<f32>());
            }
        });
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}
