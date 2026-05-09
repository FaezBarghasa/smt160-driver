# 🔬 SMT160-Driver Technical Reference Manual

This document provides a deep-dive into the mathematical foundations, signal processing algorithms, and timing analysis used in the `smt160-driver`.

## 📐 Mathematical Foundation

The SMT160 encodes temperature $T$ into duty cycle $D$ via a linear transfer function:

$$D = 0.320 + 0.00470 \times T$$

To solve for $T$:

$$T = \frac{D - 0.320}{0.00470} = (D - 0.320) \times 212.765957$$

### Inverse Step Constant
We use a pre-calculated **Inverse Step Constant** ($212.765957...$) stored as an `I32F32` fixed-point number. This transforms the division operation into a high-speed multiplication, which is deterministic and efficient on embedded CPUs.

---

## ⚡ Signal Processing Pipeline

### 1. Jitter-Free Capture
The driver relies on hardware timer input capture. By measuring $T_{active}$ and $T_{period}$ in a single hardware-gated sequence, we eliminate common-mode clock jitter from the duty cycle calculation.

### 2. Adaptive EWMA Filtering
We implement an Exponentially Weighted Moving Average (EWMA) filter with dynamic $\alpha$:

$$Y_n = \alpha X_n + (1 - \alpha) Y_{n-1}$$

- **Fast Tracking (α=0.8)**: Triggered during startup (first 16 samples) or when a large temperature jump ($>5°C$) is detected.
- **Steady State (α=0.1)**: Used during steady-state operation to suppress high-frequency thermal noise and quantization jitter.

---

## 🕒 Timing & Resolution Analysis

The theoretical resolution $\Delta T$ of the measurement is governed by the timer clock frequency $f_{clk}$ and the PWM frequency $f_{pwm} \approx 1-4kHz$.

$$\Delta T = \frac{1}{0.0047 \times f_{clk} \times T_{period}}$$

| $f_{clk}$ | Clock Period | $\Delta T$ (at 1kHz) | Bit Depth |
| :--- | :--- | :--- | :--- |
| 1 MHz | 1000 ns | 0.212 °C | ~9.5 bits |
| 8 MHz | 125 ns | 0.026 °C | ~12.5 bits |
| 72 MHz | 13.9 ns | 0.003 °C | ~15.5 bits |

> [!IMPORTANT]
> To achieve the targeted **0.05°C system accuracy**, a minimum clock frequency of **8MHz** is required to keep quantization error significantly below the sensor's inherent noise floor.

---

## 🛠️ Safety-Critical Compliance

### MISRA-C & IEC 62304 Alignment
- **Fixed-Point Arithmetic**: Prevents the use of non-deterministic floating-point hardware.
- **Boundary Checks**: All intermediate calculations for $D$ are validated against the physical limits $[0.320, 0.980]$.
- **Fail-Safe Status**: The driver provides a bit-flagged `Smt160Status` for every reading, allowing the control loop to fall back to a safe state if `SENSOR_TIMEOUT` or `OUT_OF_BOUNDS` is detected.

---

## 📊 Status & Error Taxonomy

| Flag / Error | Root Cause | System Response |
| :--- | :--- | :--- |
| `SENSOR_TIMEOUT` | No data available within watchdog period | Fail reading; trigger autonomous recovery. |
| `OUT_OF_BOUNDS` | Measured $D < 0.32$ or $D > 0.98$ | Flag signal integrity loss. |
| `JITTER_DETECTED` | Period variation > 0.5% (Configurable) | Warn of EMI or unstable clock source. |
| `InvalidSignal` | Logical error (e.g., active > period) | Drop sample; maintain filter state. |
