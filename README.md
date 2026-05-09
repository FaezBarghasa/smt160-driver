# 🌡️ SMT160-Driver (Industrial Precision Edition)

A high-integrity, deterministic Rust driver for the **SMT160** temperature sensor. Engineered for **0.05°C precision** in safety-critical industrial environments and safety-regulated applications.

[![Crates.io](https://img.shields.io/crates/v/smt160-driver.svg)](https://crates.io/crates/smt160-driver)
[![Docs.rs](https://docs.rs/smt160-driver/badge.svg)](https://docs.rs/smt160-driver)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](#license)

---

## 📚 Documentation index

| Document | Description |
| :--- | :--- |
| [Architecture](Architecture.md) | Deep dive into the driver's clean architecture and state machines. |
| [RTIC 2.1 Setup](RTIC2_setup_instruction.md) | Integration guide for Real-Time Interrupt-driven Concurrency. |
| [Baremetal Setup](baremetal_setup_instruction.md) | Guide for `no_std` environments without an executor. |
| [User Manual](User_Manual.md) | Wiring, calibration, and operational instructions for technicians. |
| [Technical Manual](Technical_Manual.md) | Mathematical derivations and performance analysis for engineers. |
| [Diagrams](Diagrams.md) | Centralized repository for all architectural and sequence diagrams. |

---

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
- **📉 Adaptive EWMA Filtering**: Real-time noise rejection that automatically adjusts responsiveness based on thermal transients (α=0.1 to 0.8).
- **🛡️ Safety Guards**: Automatic boundary validation and jitter detection.
- **⚡ Trait-Injected HAL**: Hardware-agnostic design; implement `Smt160Hal` for any MCU (STM32F1 supported out-of-box).
- **📊 Observability**: Real-time status reporting via bit-flags (Jitter, Timeout, Out-of-Bounds).

---

## 🛠️ Quick Start

### Installation

Add the following to your `Cargo.toml`:

```toml
[dependencies]
smt160-driver = "0.1.0"
fixed = "1.27.0"
```

### Usage (e.g., STM32F103 with DMA)

```rust
use smt160_driver::{Smt160Driver, Config};
use smt160_driver::hal::stm32f1_dma::Stm32F1DmaHal;

// 1. Initialize the Hardware Adapter
let hal = Stm32F1DmaHal::new(tim2, dma1_ch7);

// 2. Create the driver and initialize hardware
let mut driver = Smt160Driver::new(hal, Config::industrial())
    .init(72_000_000) // Clock frequency in Hz
    .expect("Hardware init failed");

// 3. Read temperature (non-blocking polling)
if let Some(temperature) = driver.read_temperature() {
    let temp_f32: f32 = temperature.to_num();
    println!("Temperature: {:.3} °C | Status: {:?}", temp_f32, driver.status());
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

## 🤝 Contributing

Contributions are welcome! Please ensure that any changes maintain the `no_std` compatibility and include relevant documentation updates. For major architectural changes, please open an issue first to discuss the design.

---

### License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.
