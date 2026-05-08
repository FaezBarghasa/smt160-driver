#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;
use rtic::app;
use smt160_driver::{Smt160, Ready, Smt160Status};
use stm32f1xx_hal::{
    pac,
    prelude::*,
};

#[app(device = pac, dispatchers = [SPI1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        pub driver: Smt160<Ready, pac::TIM2, stm32f1xx_hal::dma::dma1::C5>,
    }

    #[local]
    struct Local {}

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

        defmt::info!("System Clock: {} Hz", clocks.sysclk().to_Hz());

        let mut gpioa = cx.device.GPIOA.split(&mut rcc);
        let _pin = gpioa.pa1.into_floating_input(&mut gpioa.crl);

        // Static buffer for DMA double-buffering
        static mut DMA_BUFFER: [u32; 4] = [0; 4];

        // Enable peripheral clocks manually as required by PAC access
        let rcc_pac = unsafe { &*pac::RCC::ptr() };
        rcc_pac.apb1enr().modify(|_, w| w.tim2en().set_bit());
        rcc_pac.ahbenr().modify(|_, w| w.dma1en().set_bit());

        // Initialize driver
        let dma1 = cx.device.DMA1.split(&mut rcc);
        let driver = Smt160::new(cx.device.TIM2, dma1.5, unsafe { &mut DMA_BUFFER })
            .init(&clocks)
            .expect("Clock validation failed");

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
