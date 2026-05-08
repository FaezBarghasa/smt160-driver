# 🏗️ Baremetal Setup Instructions for SMT160-Driver

This guide provides instructions for integrating the `smt160-driver` into a baremetal Rust project (`no_std`) without a high-level async runtime or when using custom polling loops.

## 🎯 Prerequisites

Before you begin, ensure you have:
-   A Rust toolchain configured for embedded development (e.g., `thumbv7m-none-eabi`).
-   A timer peripheral capable of measuring pulse widths (Input Capture mode).
-   `critical-section` implementation for your platform (if using Flash or atomic features).

## 📦 Installation

Add the `smt160-driver` to your `Cargo.toml`:

```toml
[dependencies]
smt160-driver = "0.1.0"
fixed = { version = "1.27.0", features = ["az"] }
```

## ⚙️ Hardware Integration Layer

If you are not using a pre-supported platform (like STM32F1), you must implement the `CaptureDevice` trait.

```rust
use smt160_driver::platform::CaptureDevice;

pub struct MyCustomHardware;

impl CaptureDevice for MyCustomHardware {
    type Error = MyError;

    fn get_capture_data(&self) -> (u64, u64) {
        // Read raw period and active ticks from hardware registers
        let period = read_reg(TIMER_PERIOD);
        let active = read_reg(TIMER_ACTIVE);
        (period as u64, active as u64)
    }

    async fn wait_for_new_data(&mut self) -> Result<(), Self::Error> {
        // In a baremetal environment without an executor, this might 
        // poll a 'ready' bit or wait for an interrupt flag.
        while !is_data_ready() {
            core::hint::spin_loop();
        }
        Ok(())
    }
}
```

## 🚀 Usage Pattern: High-Precision Polling

In baremetal systems where low latency is critical, you can use the `Smt160BlockingDriver` which avoids the overhead of async state machines.

```rust
use smt160_driver::decoder::Smt160Decoder;
use smt160_driver::driver_blocking::Smt160BlockingDriver;

fn main() {
    // 1. Initialize logic engine (e.g. 8MHz timer)
    let decoder = Smt160Decoder::new_standalone(8);
    
    // 2. Wrap hardware and timestamp source
    let mut sensor = Smt160BlockingDriver::new(
        pin, 
        || get_timer_ticks(), 
        decoder
    );

    loop {
        // 3. Perform a blocking measurement with a 100ms timeout
        match sensor.read_temperature_with_timeout(800_000) {
            Ok(reading) => {
                // temperature is in reading.temperature_celsius
            }
            Err(e) => handle_error(e),
        }
    }
}
```

## ✨ Key Architectural Considerations

> [!TIP]
> **Deterministic Execution**: The driver uses fixed-point arithmetic, which is significantly faster and more predictable on MCUs without an FPU (like Cortex-M0/M3).

> [!WARNING]
> **Interrupt Latency**: If using the polling driver, high interrupt load on the MCU can introduce jitter into the measurement. For 0.05°C precision, ensure your `get_timer_ticks()` source is stable and consider using the `read_temperature_high_precision()` method which wraps the loop in a critical section.

---

## 📐 Accuracy vs. Clock Frequency

To achieve industrial-grade precision, ensure your timer frequency meets the resolution requirements:

| Frequency | Resolution | Status |
| :--- | :--- | :--- |
| 1 MHz | 0.21°C | ❌ Low Precision |
| 8 MHz | 0.026°C | ✅ Standard |
| 72 MHz | 0.003°C | ✅ Ultra High |