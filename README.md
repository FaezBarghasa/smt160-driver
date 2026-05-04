# SMT160-Driver

A high-precision, fixed-point, and async-friendly Rust driver for the **SMT160** temperature sensor.

[![Crates.io](https://img.shields.io/crates/v/smt160-driver.svg)](https://crates.io/crates/smt160-driver)
[![Docs.rs](https://docs.rs/smt160-driver/badge.svg)](https://docs.rs/smt160-driver)

## Overview

The SMT160 is a high-accuracy temperature sensor that outputs a Pulse Width Modulated (PWM) signal. The temperature is encoded in the duty cycle of the signal according to the formula:

**DC = 0.320 + 0.00470 * t**

This driver provides a robust, `no_std` implementation that handles the complexities of timing-critical PWM decoding on microcontrollers.

## Key Features

- **Fixed-Point Arithmetic**: Uses `I16F16` for all calculations, ensuring deterministic performance without floating-point hardware requirements.
- **Passive Logic Core**: The decoder is a state machine that accepts timestamps, making it compatible with both Interrupt-driven (RTIC) and Polled architectures.
- **Async Support**: Native `embedded-hal-async` implementation for modern async/await firmware.
- **Industrial Failsafes**:
  - **Jitter Filtering**: Discards readings that deviate more than 1.5°C from the rolling average.
  - **Frequency Watchdog**: Detects sensor failures or signal degradation by monitoring frequency shifts (>10%).
  - **Stability Counter**: Requires 5 consecutive valid pulses before yielding the first result.
  - **Thermal Bounds**: Validates readings against the SMT160 industrial range (-45°C to 130°C).

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
smt160-driver = "0.1.0"
fixed = "1.24.0"
```

## Quick Start (Async)

```rust
use smt160_driver::driver_async::Smt160Async;

// Assume 'pin' implements embedded_hal_async::digital::Wait
// Assume 'timer' returns u64 microseconds
let mut sensor = Smt160Async::new(pin, || timer.now_us());

match sensor.read_temperature().await {
    Ok(temp) => println!("Temperature: {} °C", temp),
    Err(e) => eprintln!("Error: {}", e),
}
```

## RTIC Integration

For high-precision applications, it is recommended to use the `Smt160Decoder` inside a Timer Input Capture interrupt. See the full example in `examples/f103_rtic.rs`.

```rust
// Inside TIM interrupt
if is_capture_event {
    let timestamp = capture_register();
    if let Ok(Some(temp)) = decoder.push_edge(is_rising, timestamp) {
        shared_temp.lock(|t| *t = Some(temp));
    }
}
```

## Architecture

The project is structured into three layers:
1. **`decoder.rs`**: The pure logic engine. No I/O.
2. **`driver_async.rs`**: High-level async wrapper for `embedded-hal-async`.
3. **`i2c_telemetry.rs`**: Thread-safe structures for sharing data between sensor tasks and communication interfaces.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.
