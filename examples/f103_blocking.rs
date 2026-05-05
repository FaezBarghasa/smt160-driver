//! Blocking Example for SMT160 on STM32F103 (Bluepill).
//!
//! This example demonstrates the synchronous (blocking) driver usage.
//! It uses a hardware timer to provide a microsecond timestamp source
//! and polls a GPIO pin in a busy-loop to detect temperature signal edges.
//!
//! ## Hardware Configuration
//! - **MCU**: STM32F103C8T6
//! - **Pin**: PA0 (Floating Input)
//! - **Timer**: TIM2 configured as a 1µs free-running counter.
//!
//! ## Wiring
//! - SMT160 VCC -> 3.3V
//! - SMT160 GND -> GND
//! - SMT160 OUT -> PA0

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use smt160_driver::driver_blocking::Smt160Blocking;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    timer::CounterUs,
};
use cortex_m_rt::entry;
use defmt::{info, error};

#[entry]
fn main() -> ! {
    // 1. Core and Device Peripherals
    let cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // 2. Setup Clocks (72MHz)
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr
        .use_hse(8.MHz())
        .sysclk(72.MHz())
        .pclk1(36.MHz())
        .freeze(&mut flash.acr);

    // 3. Setup GPIO
    let mut gpioa = dp.GPIOA.split();
    let sensor_pin = gpioa.pa0.into_floating_input(&mut gpioa.crl);

    // 4. Setup Timer for microsecond timestamps
    // We use CounterUs which provides a simple way to get 'now()' in microseconds.
    let mut timer = dp.TIM2.counter_us(&clocks);
    timer.start(1.secs()).unwrap(); // Wrap every second, but we only need it for delta

    // 5. Initialize the Blocking Driver
    // We provide the pin and a closure that returns the current microsecond count.
    // Note: Since CounterUs is 16-bit on F1, we might need to handle wrap-around 
    // or use a 32-bit source. For this example, we'll use a simpler approach 
    // by tracking overflows manually or using a HAL provided 32-bit-like source if available.
    // However, stm32f1xx-hal's CounterUs for TIM2 is 16-bit.
    
    // Better: Use the DWT (Data Watchpoint and Trace) for a 32-bit cycle counter 
    // and convert to microseconds.
    let mut dcb = cp.DCB;
    let mut dwt = cp.DWT;
    dwt.enable_cycle_counter(&mut dcb);
    
    let get_time_us = || {
        let cycles = dwt.get_cycle_count();
        // 72MHz = 72 cycles per microsecond
        (cycles as u64) / 72
    };

    let mut smt160 = Smt160Blocking::new(sensor_pin, get_time_us);

    info!("SMT160 Blocking Driver Example Started");

    loop {
        // Read temperature with 500ms timeout
        match smt160.read_temperature(500_000) {
            Ok(temp) => {
                // Fixed-point I16F16 can be converted or printed directly if defmt is configured.
                info!("Temperature: {} C", temp.to_num::<f32>());
            }
            Err(e) => {
                error!("Driver Error: {}", e);
            }
        }

        // Wait a bit before next reading
        cortex_m::asm::delay(clocks.sysclk().raw() / 2);
    }
}
