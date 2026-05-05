# SMT160-Driver (High-Accuracy Edition)

A high-precision, fixed-point, and async-friendly Rust driver for the **SMT160** temperature sensor, targeting **0.05°C precision**.

[![Crates.io](https://img.shields.io/crates/v/smt160-driver.svg)](https://crates.io/crates/smt160-driver)
[![Docs.rs](https://docs.rs/smt160-driver/badge.svg)](https://docs.rs/smt160-driver)

## Overview

The SMT160 is a high-accuracy temperature sensor that outputs a Pulse Width Modulated (PWM) signal. The temperature is encoded in the duty cycle of the signal according to the formula:

**DC = 0.320 + 0.00470 * t**

This driver provides a robust, `no_std` implementation that handles the complexities of timing-critical PWM decoding with industrial-grade reliability.

## Key Features

- **High Precision (0.05°C Target)**: Optimized for high-resolution timers (up to 72MHz capture) with sub-microsecond timestamping.
- **Fixed-Point Engine**: Uses `I32F32` for intermediate calculations and `I16F16` for results, ensuring deterministic performance without an FPU.
- **Clock-Aware Decoder**: The passive logic core supports custom clock frequencies, allowing for raw tick-based decoding with zero rounding loss.
- **Async Native**: Supports `embedded-hal-async` with native Rust 2024 async-in-trait support.
- **Industrial Failsafes**:
  - **16-Sample Moving Average**: Smooths readings while maintaining responsiveness.
  - **Outlier Rejection**: Discards noise spikes (>2.0°C from rolling average).
  - **Critical Section Polling**: Provides jitter-free measurement for timing-sensitive blocking applications.
  - **Thermal Bounds**: Validates readings against the SMT160 range (-45°C to 130°C).

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
smt160-driver = "0.1.0"
fixed = { version = "1.27.0", features = ["az"] }
```

## Usage

### High-Accuracy Async (e.g. ESP32/RTOS)

```rust
use smt160_driver::decoder::Smt160Decoder;
use smt160_driver::driver_async::Smt160Async;

// 1. Configure decoder for your timer frequency (e.g., 1MHz for microseconds)
let decoder = Smt160Decoder::with_clock(1);

// 2. Initialize driver with pin and timestamp source
let mut sensor = Smt160Async::new(pin, || timer.now_us(), decoder);

// 3. Read temperature (averaged over 16 samples)
match sensor.read_temperature().await {
    Ok(temp) => println!("Temperature: {} °C", temp.to_num::<f32>()),
    Err(e) => eprintln!("Error: {:?}", e),
}
```

### High-Accuracy Blocking (e.g. STM32 Bluepill)

For maximum precision, use the `read_temperature_precision` method which disables interrupts during a single cycle measurement.

```rust
use smt160_driver::decoder::Smt160Decoder;
use smt160_driver::driver_blocking::Smt160Blocking;

// Use 72MHz ticks for ~0.003°C resolution
let decoder = Smt160Decoder::with_clock(72);
let mut sensor = Smt160Blocking::new(pin, || dwt.cycle_count() as u64, decoder);

match sensor.read_temperature_precision() {
    Ok(temp) => info!("Precise Temp: {} °C", temp.to_num::<f32>()),
    Err(e) => error!("Error: {:?}", e),
}
```

## Hardware Guide: Resolution vs Clock

To achieve the **0.05°C accuracy** target, your capture clock must be high enough to resolve small duty cycle shifts:

| Clock Frequency | Resolution | Target Met? |
|-----------------|------------|-------------|
| 1 MHz (1µs)     | ~0.210°C   | ❌ No        |
| 8 MHz (125ns)   | ~0.026°C   | ✅ Yes       |
| 72 MHz (13ns)   | ~0.003°C   | ✅ Yes (Ultra) |

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.
