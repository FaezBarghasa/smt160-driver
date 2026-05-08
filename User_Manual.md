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

The driver returns a `Reading` structure which contains two fields:

1.  **`temperature_celsius`**: The current temperature in °C.
2.  **`status`**: A bitmask indicating the health of the sensor signal.

### Status Flags Table

| Flag | Meaning | Action Required |
| :--- | :--- | :--- |
| `OK` | Nominal | None. Reading is valid. |
| `SIGNAL_LOSS` | No Signal | Check wiring and power to sensor. |
| `FREQUENCY_ERROR` | Bad Timing | Check for interference or clock configuration. |
| `OUT_OF_BOUNDS` | Temp > 130°C | Sensor may be damaged or in thermal runaway. |
| `JITTER_ALERT` | Unstable | Reduce electrical noise; check decoupling. |

---

## 📉 Calibration Guide

While the SMT160 is factory-calibrated, the `smt160-driver` supports **5-point piecewise linear interpolation** for field calibration.

### When to Calibrate?
- If the sensor is used at the extremes of its range (-45°C or +130°C).
- If there is a constant offset due to long cable runs or PCB thermal leakage.

### How to Calibrate?
1.  Place the sensor in a known reference environment (e.g., Ice Bath for 0°C).
2.  Note the measured temperature vs. reference temperature.
3.  Update the `CalibrationProfile` in your software with the measured points.

---

## ❓ Troubleshooting

### Reading is always -45°C or +130°C
- **Cause**: Signal is stuck High or Low.
- **Check**: Ensure the sensor is powered and the `OUT` pin is connected to the correct MCU input.

### Reading is "jumpy" or noisy
- **Cause**: High jitter on the PWM signal.
- **Fix**: Check for nearby high-power AC lines. Ensure ground loops are minimized. The driver's internal EWMA filter will attempt to smooth this, but `JITTER_ALERT` will trigger if noise is excessive.

### `Smt160Error::Timeout`
- **Cause**: The driver did not detect any PWM edges within the expected window.
- **Check**: Verify the sensor's PWM output with an oscilloscope or logic analyzer.
