# 🏗️ Baremetal Setup Instructions for SMT160-Driver

This guide provides instructions for integrating the `smt160-driver` into a baremetal Rust project (`no_std`) using the new trait-injected architecture.

## 🎯 Prerequisites

Before you begin, ensure you have:
- A Rust toolchain configured for embedded development (e.g., `thumbv7m-none-eabi`).
- A hardware timer/DMA subsystem capable of zero-jitter pulse capture.

## 📦 Installation

Add the `smt160-driver` to your `Cargo.toml`:

```toml
[dependencies]
smt160-driver = "0.1.0"
fixed = "1.27.0"
```

## ⚙️ Hardware Abstraction Layer (HAL)

To use the driver on any MCU, implement the `Smt160Hal` trait. This trait serves as the contract between the hardware registers and the generic decoding logic.

```rust
use smt160_driver::hal::{Smt160Hal, CapturedEdge};
use smt160_driver::error::Smt160Error;

pub struct MyMcuHal {
    // Peripheral ownership (e.g., Timer, DMA)
}

impl Smt160Hal for MyMcuHal {
    fn setup(&mut self, freq_hz: u32) -> Result<(), Smt160Error> {
        // Configure Timer for PWM Input Mode
        // Configure DMA for circular capture
        Ok(())
    }

    fn is_new_data_available(&self) -> bool {
        // Check DMA Half-Transfer or Transfer-Complete flags
        true 
    }

    fn read_raw(&self) -> CapturedEdge {
        // Return the latest captured ticks
        CapturedEdge {
            period_ticks: 10000, // Total cycle duration
            high_ticks: 4375,   // High phase duration
        }
    }
}
```

## 🚀 Usage Pattern: Polling Loop

In baremetal systems, the driver is typically polled in the main loop or a periodic interrupt.

```rust
use smt160_driver::{Smt160Driver, Config};

fn main() {
    let my_hal = MyMcuHal::new();
    
    // 1. Create the driver with a specific configuration
    let mut driver = Smt160Driver::new(my_hal, Config::industrial());
    
    // 2. Initialize (transitions Uninitialized -> Ready)
    let mut driver = driver.init(72_000_000).expect("Hardware init failed");

    loop {
        // 3. Poll for new readings
        if let Some(temperature) = driver.read_temperature() {
            // temperature is a fixed-point I32F32 value
            let temp_f32: f32 = temperature.to_num();
        }
    }
}
```

## ✨ Key Architectural Considerations

> [!TIP]
> **Adaptive Filtering**: The driver automatically applies an adaptive EWMA filter. During startup or rapid thermal transients (>5°C), it uses a fast-track $\alpha=0.8$. Once stabilized, it switches to $\alpha=0.1$ for maximum noise rejection.

> [!IMPORTANT]
> **Fixed-Point Math**: All calculations use `I32F32` fixed-point math. This ensures deterministic performance on MCUs without an FPU and prevents rounding errors common with floating-point emulators.

---

## 📐 Accuracy vs. Clock Frequency

To achieve the 0.05°C precision target, your timer frequency must provide sufficient resolution:

| Frequency | Resolution | Status |
| :--- | :--- | :--- |
| 1 MHz | 0.21°C | ❌ Low Precision |
| 8 MHz | 0.026°C | ✅ Standard |
| 72 MHz | 0.003°C | ✅ Ultra High |