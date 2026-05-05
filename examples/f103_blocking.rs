//! Blocking Example for SMT160 on STM32F103 (Bluepill).
//!
//! This example demonstrates the synchronous (blocking) driver usage.
//! It uses a hardware timer to provide a microsecond timestamp source
//! and polls a GPIO pin in a busy-loop to detect temperature signal edges.
//!
//! ## Hardware Configuration
//! - **MCU**: STM32F103C8T6
//! - **Pin**: PA0 (Floating Input)
//! - **Timer**: DWT (Data Watchpoint and Trace) cycle counter for microseconds.
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
};
use cortex_m_rt::entry;
use cortex_m::peripheral::DWT;
use defmt::{info, error};

#[entry]
fn main() -> ! {
    // 1. Core and Device Peripherals
    let mut cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // 2. Setup Clocks (72MHz)
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    let mut clocks = rcc.freeze(
        stm32f1xx_hal::rcc::Config::hse(8.MHz())
            .sysclk(72.MHz())
            .pclk1(36.MHz()),
        &mut flash.acr,
    );

    // 3. Setup GPIO
    let mut gpioa = dp.GPIOA.split(&mut clocks);
    let sensor_pin = gpioa.pa0.into_floating_input(&mut gpioa.crl);

    // 4. Setup DWT for microsecond timestamps
    cp.DWT.enable_cycle_counter();
    
    // 72MHz system clock means 72 cycles per microsecond.
    let get_time_us = || {
        let cycles = DWT::cycle_count();
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

        // Wait a bit before next reading (~500ms)
        cortex_m::asm::delay(72_000_000 / 2);
    }
}
