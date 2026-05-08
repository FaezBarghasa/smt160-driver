#![no_std]
#![no_main]

//! SMT160 DMA-based Real-time Test Example
//!
//! Streams filtered temperature readings frequently via defmt (RTT) for zero-cost real-time monitoring.

use defmt_rtt as _;
use panic_halt as _;
use smt160_driver::decoder::Smt160Decoder;
use stm32f1xx_hal::{
    prelude::*,
    pac,
};

#[rtic::app(device = pac, dispatchers = [EXTI0])]
mod app {
    use super::*;
    use fixed::types::I16F16;

    #[shared]
    struct Shared {
        decoder: Smt160Decoder,
        current_temp: Option<I16F16>,
    }

    #[local]
    struct Local {
        dma_buffer: &'static mut [u16; 64],
        dma1: pac::DMA1,
    }

    #[init(local = [buf: [u16; 64] = [0; 64]])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();
        
        // Setup RCC with External Crystal (HSE) at 72MHz
        let mut rcc = rcc.freeze(
            stm32f1xx_hal::rcc::Config::hse(8.MHz())
                .sysclk(72.MHz())
                .pclk1(36.MHz()),
            &mut flash.acr,
        );

        let mut gpioa = cx.device.GPIOA.split(&mut rcc);
        let _pa0 = gpioa.pa0.into_floating_input(&mut gpioa.crl);

        let tim2 = cx.device.TIM2;
        let dma1 = cx.device.DMA1;

        unsafe {
            let rcc_ptr = &*pac::RCC::ptr();
            rcc_ptr.ahbenr().modify(|_, w| w.dma1en().set_bit());
        }

        tim2.psc().write(|w| unsafe { w.psc().bits(0) });
        tim2.arr().write(|w| unsafe { w.arr().bits(0xFFFF) });

        tim2.ccmr1_input().modify(|_, w| w.cc1s().ti1());
        tim2.ccmr1_input().modify(|_, w| w.cc2s().ti1());
        tim2.ccer().modify(|_, w| w.cc1p().clear_bit());
        tim2.ccer().modify(|_, w| w.cc2p().set_bit());

        tim2.smcr().modify(|_, w| unsafe { w.ts().bits(0b101) });
        tim2.smcr().modify(|_, w| unsafe { w.sms().bits(0b100) });

        tim2.dcr().write(|w| unsafe { w.dba().bits(13).dbl().bits(1) });
        tim2.ccer().modify(|_, w| w.cc1e().set_bit().cc2e().set_bit());
        tim2.dier().modify(|_, w| w.cc1de().set_bit());
        tim2.cr1().modify(|_, w| w.cen().set_bit());

        let ch7 = dma1.ch7();
        ch7.cr().modify(|_, w| w.en().clear_bit());
        ch7.par().write(|w| unsafe { w.pa().bits(tim2.dmar().as_ptr() as u32) });
        ch7.mar().write(|w| unsafe { w.ma().bits(cx.local.buf.as_ptr() as u32) });
        ch7.ndtr().write(|w| unsafe { w.ndt().bits(64) });
        
        ch7.cr().modify(|_, w| {
            w.minc().set_bit().pinc().clear_bit().msize().bits16()
             .psize().bits16().circ().set_bit().dir().clear_bit()
             .teie().set_bit().htie().set_bit().tcie().set_bit()
        });
        ch7.cr().modify(|_, w| w.en().set_bit());

        defmt::info!("SMT160 RTIC DMA Test Started!");

        (
            Shared { 
                decoder: Smt160Decoder::new_standalone(72),
                current_temp: None,
            },
            Local { dma_buffer: cx.local.buf, dma1 },
        )
    }

    #[task(binds = DMA1_CHANNEL7, local = [dma1, dma_buffer], shared = [decoder, current_temp])]
    fn dma1_ch7_irq(cx: dma1_ch7_irq::Context) {
        let isr = cx.local.dma1.isr().read();
        let mut process_range = None;

        if isr.htif7().bit_is_set() {
            cx.local.dma1.ifcr().write(|w| w.chtif7().set_bit());
            process_range = Some(0..32);
        } else if isr.tcif7().bit_is_set() {
            cx.local.dma1.ifcr().write(|w| w.ctcif7().set_bit());
            process_range = Some(32..64);
        }

        if let Some(range) = process_range {
            let mut batch = [0u32; 16];
            let buf = &cx.local.dma_buffer[range];
            for i in 0..16 {
                let period = buf[i * 2] as u32;
                let active = buf[i * 2 + 1] as u32;
                batch[i] = (period << 16) | active;
            }
            
            let mut decoder_lock = cx.shared.decoder;
            let mut temp_lock = cx.shared.current_temp;
            
            decoder_lock.lock(|decoder| {
                if let Ok(Some(reading)) = decoder.process_batch(&batch, smt160_driver::config::DUTY_CYCLE_OFFSET, smt160_driver::config::INVERSE_STEP_CONSTANT) {
                    temp_lock.lock(|temp| *temp = Some(reading.temperature_celsius));
                }
            });
        }
    }

    #[idle(shared = [current_temp])]
    fn idle(mut cx: idle::Context) -> ! {
        let mut last_temp = None;
        loop {
            // Retrieve temperature without race conditions
            cx.shared.current_temp.lock(|temp| {
                if let Some(t) = *temp {
                    if Some(t) != last_temp {
                        defmt::info!("Filtered Temp: {} C", t);
                        last_temp = Some(t);
                    }
                }
            });
            cortex_m::asm::wfi();
        }
    }
}
