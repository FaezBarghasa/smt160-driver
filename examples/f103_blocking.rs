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

    // 4. Setup DWT for cycle-accurate timestamps
    cp.DWT.enable_cycle_counter();
    
    // 72MHz system clock (1 tick = 13.88ns)
    let get_ticks = || {
        DWT::cycle_count() as u64
    };

    use smt160_driver::decoder::Smt160Decoder;
    let decoder = Smt160Decoder::with_clock(72);
    let mut smt160 = Smt160Blocking::new(sensor_pin, get_ticks, decoder);

    info!("SMT160 Blocking Driver Example Started");

    loop {
        // Option 1: High-precision reading (blocks all other tasks)
        // match smt160.read_temperature_precision() {
        
        // Option 2: Standard reading with timeout (36M ticks = 500ms)
        match smt160.read_temperature(36_000_000) {
            Ok(reading) => {
                info!("Temperature: {} C, Status: {:?}", reading.value.to_num::<f32>(), reading.status);
            }
            Err(e) => {
                error!("Driver Error: {}", e);
            }
        }

        // Wait a bit before next reading (~500ms)
        cortex_m::asm::delay(72_000_000 / 2);
    }
}
