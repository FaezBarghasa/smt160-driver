# 🌡️ SMT160-Driver (Industrial Precision Edition)

A high-integrity, deterministic Rust driver for the **SMT160** temperature sensor. Engineered for **0.05°C precision** in safety-critical industrial environments and safety-regulated applications.

[![Crates.io](https://img.shields.io/crates/v/smt160-driver.svg)](https://crates.io/crates/smt160-driver)
[![Docs.rs](https://docs.rs/smt160-driver/badge.svg)](https://docs.rs/smt160-driver)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](#license)

---

## 🏗️ Architecture: Self-Documenting & Clean

This driver implements a **Self-Documenting Clean Architecture**, strictly decoupling core mathematical state machines from hardware-specific capture logic. This ensures a single logic engine can be audited once and deployed across **STM32, ESP32, nRF, or virtualized environments**.

> [!TIP]
> **Deterministic Performance**: All calculations use `I32F32` and `I16F16` fixed-point arithmetic, ensuring bit-perfect consistency on Cortex-M devices without an FPU.

---

## 📡 Theory of Operation

The SMT160 sensor outputs a pulse-width modulated (PWM) signal where the temperature is encoded in the duty cycle ($D$):

$$D = 0.320 + 0.00470 \times T [°C]$$

Our driver decodes this signal by measuring the **Period** ($T_p$) and **Active High** ($T_a$) durations using high-resolution timers. The temperature is then derived using a high-performance inverse step constant:

$$T = \frac{\frac{T_a}{T_p} - 0.320}{0.00470}$$

---

## ✨ Key Features

- **🎯 Industrial Precision**: Targeted 0.05°C accuracy with support for high-resolution timers (up to 72MHz).
- **📉 Piecewise Calibration**: Integrated support for multi-point linear interpolation to correct sensor non-linearity.
- **🛡️ Safety Guards**: Automatic boundary validation (0.320-0.980 Duty Cycle) and frequency drift monitoring (500Hz - 5kHz).
- **⚡ Async Native**: Zero-overhead support for `embedded-hal-async` and multi-tasking RTOS environments (e.g., RTIC).
- **📊 Observability**: Real-time health metrics including jitter RMS, frequency stability tracking, and bit-flagged status reporting.

---

## 🛠️ Quick Start

### Installation

Add the following to your `Cargo.toml`:

```toml
[dependencies]
smt160-driver = "0.1.0"
fixed = { version = "1.27.0", features = ["az"] }
```

### High-Precision Usage (Async/RTIC)

```rust
use smt160_driver::{Smt160Driver, Reading};
use smt160_driver::config::StaticConfiguration;

// 1. Initialize the driver with hardware-specific capture (e.g., STM32)
let mut sensor = Smt160Driver::new(
    StaticConfiguration, 
    stm32_capture_device, 
    72 // Timer clock in MHz for high-resolution edge detection
);

// 2. Perform a non-blocking high-precision reading
match sensor.read_temperature_celsius().await {
    Ok(reading) => {
        let temp_f32: f32 = reading.temperature_celsius.to_num();
        println!("Temperature: {:.3} °C | Status: {:?}", temp_f32, reading.status);
    }
    Err(e) => eprintln!("Hardware System Fault: {}", e),
}
```

---

## 📐 Precision & Performance

| Clock Frequency | Resolution | Industrial Target | Use Case |
| :--- | :--- | :--- | :--- |
| **1 MHz** (1µs) | ~0.210°C | ❌ No | Low-power indicators |
| **8 MHz** (125ns) | ~0.026°C | ✅ Yes | Standard HVAC Control |
| **72 MHz** (13ns) | ~0.003°C | ✅ Yes (Ultra) | Laboratory Calibration |

---

## 🛡️ Safety & Integrity Standards

This driver is designed with **MISRA-C** and **IEC 62304** principles in mind:

- **No Panics**: All internal paths are `Result`-based; indexing is checked or uses compile-time guarantees.
- **No Floating Point**: Prevents non-deterministic behavior and rounding errors on embedded hardware.
- **No Allocations**: Operates entirely on the stack or statically allocated memory (`no_std`).
- **Static Generics**: Uses Zero-Cost Abstractions for platform drivers, avoiding dynamic dispatch (vtable) overhead and latency.

> [!IMPORTANT]
> **Boundary Protection**: The driver immediately rejects signals with duty cycles below 0.320 or above 0.980, detecting wiring faults or sensor hardware degradation before they impact control loops.

---

### License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.
