# 📘 SMT160 Sensor User Manual

This manual provides instructions for connecting, calibrating, and troubleshooting the SMT160 temperature sensor using the `smt160-driver`.

## 🔌 Hardware Connection

The SMT160 is a three-pin sensor. Proper wiring is essential for high-accuracy readings.

| Pin | Function | Description |
| :--- | :--- | :--- |
| **VCC** | Supply | 3.0V to 7.0V (Standard 3.3V or 5V) |
| **GND** | Ground | Common system ground |
| **OUT** | Signal | PWM output (Connect to Timer Input pin) |

> [!IMPORTANT]
> **Decoupling**: Place a 100nF ceramic capacitor as close as possible to the VCC and GND pins of the sensor to minimize electrical noise on the PWM signal.

---

## 🌡️ Understanding Readings

The driver returns a temperature reading (as a fixed-point `I32F32`) and maintains a `status` bitmask indicating the health of the sensor signal.

### Status Flags Table

| Flag | Meaning | Action Required |
| :--- | :--- | :--- |
| `SENSOR_TIMEOUT` | No Signal | Check wiring and power to sensor. |
| `OUT_OF_BOUNDS` | Integrity Lost | Signal is outside physical limits (Duty Cycle < 0.32 or > 0.98). |
| `JITTER_DETECTED` | Unstable Signal | Period variation > 0.5%. Reduce electrical noise; check decoupling. |

---

## 📉 Signal Processing: Adaptive EWMA

The driver uses an **Adaptive Exponentially Weighted Moving Average (EWMA)** filter to provide smooth readings without sacrificing responsiveness. 

- If the temperature is stable, it uses a high-smoothing factor (α=0.1).
- If a sudden jump (>5°C) or startup is detected, it automatically switches to a fast-tracking mode (α=0.8).

---

## ❓ Troubleshooting

### Reading returns `None`
- **Cause**: No new data has been captured since the last poll.
- **Action**: Increase your polling frequency or check if the sensor is still pulsing.

### `SENSOR_TIMEOUT` is set
- **Cause**: The driver's internal watchdog detected no edges.
- **Check**: Verify the sensor's PWM output with an oscilloscope. Ensure the DMA/Timer is correctly configured.

### `OUT_OF_BOUNDS` is set
- **Cause**: The measured duty cycle is physically impossible for an SMT160.
- **Fix**: Check for short circuits or heavy EMI on the signal line.

### `JITTER_DETECTED` is set
- **Cause**: High variation in the PWM period.
- **Fix**: Ensure the sensor has a 100nF decoupling capacitor. Avoid routing signal lines near high-current motor drivers or AC power lines.
