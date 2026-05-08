# 🏗️ Baremetal Setup Instructions for SMT160-Driver

This guide provides instructions for integrating the `smt160-driver` into a baremetal Rust project, typically targeting microcontrollers without an operating system (`no_std`). The driver is designed for high-integrity, deterministic operation, making it ideal for such environments.

## 🎯 Prerequisites

Before you begin, ensure you have:

-   A Rust toolchain configured for embedded development (e.g., `rustup target add thumbv7em-none-eabihf`).
-   A Hardware Abstraction Layer (HAL) for your target microcontroller that implements the `embedded-hal-async` traits, particularly for timer capture peripherals. Examples include `stm32-hal`, `esp-hal`, `rp2040-hal`, or `atsamd-hal`.
-   Basic understanding of `no_std` Rust development and asynchronous programming with `async/await`.

## 📦 Installation

Add the `smt160-driver` and its `fixed` dependency to your project's `Cargo.toml`:

```toml
[dependencies]
smt160-driver = "0.1.0"
fixed = { version = "1.27.0", features = ["az"] }
# Your HAL crate, e.g., for STM32
# stm32f4xx-hal = { version = "0.1.0", features = ["stm32f401", "async"] }
# Or for ESP32
# esp-hal = { version = "0.1.0", features = ["esp32c3", "async"] }
```

Ensure your HAL crate is configured to provide `async` capabilities and implements the necessary `embedded-hal-async` traits for timer input capture.

## ⚙️ Hardware Integration

The `smt160-driver` requires a hardware-specific `CaptureDevice` that can measure the period ($T_p$) and active high ($T_a$) durations of the SMT160's PWM signal. This device must implement the `smt160_driver::CaptureDevice` trait, which typically wraps a high-resolution timer peripheral from your HAL.

The driver leverages `embedded-hal-async`, meaning your `CaptureDevice` implementation will likely involve `async` functions provided by your HAL.

## 🚀 Example Usage

Here's a generic example demonstrating how to use the `smt160-driver` in a baremetal `no_std` environment. This assumes you have an `async` runtime (e.g., a simple executor) and a `CaptureDevice` instance from your HAL.

```rust
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)] // Required for some async executors

use core::panic::PanicInfo;
use smt160_driver::{Smt160Driver, Reading};
use smt160_driver::config::StaticConfiguration;

// Assume your HAL provides an async executor and a CaptureDevice implementation
// This is a placeholder for your actual HAL and executor setup
mod my_hal {
    use smt160_driver::CaptureDevice;
    use core::future::Future;

    pub struct MyCaptureDevice;

    impl CaptureDevice for MyCaptureDevice {
        type Error = (); // Replace with your HAL's error type
        type CaptureFuture<'a> = impl Future<Output = Result<(u32, u32), Self::Error>> + 'a;

        fn capture<'a>(&'a mut self) -> Self::CaptureFuture<'a> {
            async move {
                // In a real application, this would interact with your timer peripheral
                // and await the capture of period and active high durations.
                // For demonstration, we'll return dummy values.
                // These values correspond to a duty cycle of ~0.5, which is ~38.3°C
                Ok((100_000, 50_000)) // Example: 100,000 timer ticks for period, 50,000 for active high
            }
        }
    }

    // Your async executor setup would go here
    // For simplicity, we'll omit a full executor implementation in this example.
    // You would typically use a crate like `embassy-executor` or a custom one.
    pub async fn run_async_main<F: Future>(f: F) {
        f.await
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    my_hal::run_async_main(async {
        // 1. Initialize the driver with your hardware-specific capture device
        let mut sensor = Smt160Driver::new(
            StaticConfiguration,
            my_hal::MyCaptureDevice,
            72, // Timer clock in MHz for high-resolution edge detection (e.g., 72MHz for STM32)
        );

        // 2. Perform a non-blocking high-precision reading
        match sensor.read_temperature_celsius().await {
            Ok(reading) => {
                let temp_f32: f32 = reading.temperature_celsius.to_num();
                // In a baremetal context, you might send this over UART, display on an LCD, etc.
                // For this example, we'll just conceptually "print"
                // You would use a logging framework like `defmt` here.
                // defmt::info!("Temperature: {:.3} °C | Status: {:?}", temp_f32, reading.status);
            }
            Err(e) => {
                // Handle hardware system faults
                // defmt::error!("Hardware System Fault: {:?}", e);
            }
        }
    });

    loop {
        // Your main loop might put the MCU to sleep or handle other tasks
        cortex_m::asm::wfi();
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // Implement your panic handler for baremetal
    // e.g., turn on an LED, log the error, reset the device
    loop {
        cortex_m::asm::bkpt();
    }
}
```

## ✨ Key Considerations for Baremetal

-   **Fixed-Point Arithmetic**: The driver exclusively uses `I32F32` and `I16F16` fixed-point numbers, eliminating floating-point dependencies and ensuring deterministic behavior on Cortex-M devices without an FPU.
-   **No Allocations**: The driver operates entirely on the stack or statically allocated memory, making it suitable for resource-constrained embedded systems.
-   **Error Handling**: All operations return `Result` types, preventing panics and allowing robust error management.
-   **Timer Configuration**: Correctly configuring your microcontroller's timer to provide accurate period and active-high measurements is crucial for the driver's precision. The `timer_clock_mhz` parameter in `Smt160Driver::new` should match your timer's input clock frequency.
```
```diff