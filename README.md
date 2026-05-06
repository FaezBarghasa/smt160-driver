# SMT160-Driver (Industrial Precision Edition)

A high-integrity, fixed-point, and hardware-agnostic Rust driver for the **SMT160** temperature sensor, designed for **0.05°C precision** in safety-critical industrial applications.

[![Crates.io](https://img.shields.io/crates/v/smt160-driver.svg)](https://crates.io/crates/smt160-driver)
[![Docs.rs](https://docs.rs/smt160-driver/badge.svg)](https://docs.rs/smt160-driver)

## 🏗️ Architecture: Self-Documenting & Clean

This driver implements a **Self-Documenting Clean Architecture**, separating core mathematical logic from hardware-specific capture logic. This ensures that the same logic engine can be used across STM32, ESP32, nRF, or even virtualized environments without modification.

> [!TIP]
> **Zero-FPU Requirement**: All calculations use `I32F32` and `I16F16` fixed-point arithmetic, ensuring deterministic performance on Cortex-M0/M3/M4 devices without hardware floating-point support.

## 🚀 Key Features

- **Industrial Grade Precision**: Targeted at 0.05°C accuracy with support for high-resolution timers (up to 72MHz).
- **Multi-Phase Calibration**: Supports 5-point piecewise linear interpolation for non-linear sensor correction.
- **Hardware Agnostic HAL**: Core driver is generic over the `CaptureDevice` trait.
- **Observability & Health**: Integrated monitoring for signal jitter (RMS), frequency drift, and error tracking.
- **Async & Non-Blocking**: Native support for `embedded-hal-async` and multitasking environments.

## 🛠️ Installation

Add the following to your `Cargo.toml`:

```toml
[dependencies]
smt160-driver = "0.1.0"
fixed = { version = "1.27.0", features = ["az"] }
```

## 📖 Usage Examples

### Modern Asynchronous Implementation (Generic)

```rust
use smt160_driver::{Smt160Driver, Reading};
use smt160_driver::config::StaticConfiguration;

// 1. Initialize the generic driver with hardware-specific capture
let mut sensor = Smt160Driver::new(
    StaticConfiguration, 
    stm32_capture_device, 
    72 // Timer frequency in MHz
);

// 2. Perform high-precision asynchronous reading
match sensor.read_temperature_celsius().await {
    Ok(reading) => println!("Temp: {} °C, Status: {:?}", reading.temperature_celsius.to_num::<f32>(), reading.status),
    Err(e) => eprintln!("Sensor Error: {}", e),
}
```

### High-Precision Polling (Low-Latency)

```rust
use smt160_driver::decoder::Smt160Decoder;
use smt160_driver::driver_blocking::Smt160BlockingDriver;

let decoder = Smt160Decoder::new_standalone(72);
let mut sensor = Smt160BlockingDriver::new(pin, || dwt.cycle_count() as u64, decoder);

// Perform measurement within a critical section to eliminate capture jitter
if let Ok(reading) = sensor.read_temperature_high_precision() {
    info!("Precise Temperature: {} °C", reading.temperature_celsius.to_num::<f32>());
}
```

## 📊 Precision & Hardware Requirements

| Clock Frequency | Resolution | Industrial Target | Use Case |
| :--- | :--- | :--- | :--- |
| 1 MHz (1µs) | ~0.210°C | ❌ No | Low-power indicators |
| 8 MHz (125ns) | ~0.026°C | ✅ Yes | Standard HVAC/Process Control |
| 72 MHz (13ns) | ~0.003°C | ✅ Yes (Ultra) | Laboratory Calibration |

## 🛡️ Safety & Integrity

> [!IMPORTANT]
> **Safety Guards**: The driver includes automatic boundary validation (0.320-0.980 Duty Cycle) and frequency range validation (1kHz-4kHz) to detect sensor hardware failures or wiring issues immediately.

---

### License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.
