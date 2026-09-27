# SMT160 Industrial Driver Safety Manual

This document outlines the safety features, failure modes, and autonomous recovery procedures for the `smt160-driver`.

## 1. Failure Modes & Detection

### 1.1 Sensor Timeout (`SENSOR_TIMEOUT`)
- **Detection**: No PWM pulses detected within the configured `timeout_ms` window (default 500ms).
- **Causes**: Sensor disconnection, broken wires, or hardware ESD freeze.
- **Action**: The driver sets the `SENSOR_TIMEOUT` flag in the status.

### 1.2 Out of Bounds (`OUT_OF_BOUNDS`)
- **Detection**: Decoded temperature is outside the physical range of -45°C to +130°C.
- **Causes**: Extreme thermal events or significant signal corruption.
- **Action**: The measurement is discarded, and the `OUT_OF_BOUNDS` flag is set.

### 1.3 Signal Integrity Issues
- **Detection**: High variance in raw ticks (monitored via `Diagnostics`).
- **Causes**: EMI interference, loose connectors, or cable degradation.
- **Action**: Monitor `diagnostics.std_dev()` and trigger maintenance if it exceeds a predefined threshold.

## 2. Autonomous Recovery Procedures

### 2.1 Hardware Re-initialization
The `reinit()` method provides a safe way to reset the underlying hardware (Timers, DMA) without destroying the driver instance. This is the primary mechanism for recovering from `SENSOR_TIMEOUT`.

### 2.2 Adaptive Filtering
The driver uses an adaptive EWMA filter to reject noise while maintaining fast response to real thermal events. During startup or large deviations, the filter automatically switches to a higher alpha (fast track) to reach stability quickly.

## 3. Best Practices for Industrial Safety

1.  **Watchdog Task**: Always run the driver inside an async task with a monotonic timer to monitor health.
2.  **Redundancy**: For critical systems, use two SMT160 sensors on different timer channels (Phase 3 support).
3.  **Calibration**: Perform multi-point calibration for each sensor and store coefficients in secure storage (Phase 2 support).
