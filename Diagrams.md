# 📊 SMT160-Driver Visual Documentation

This document serves as a centralized repository for all Mermaid diagrams describing the architecture, signal processing flows, and state transitions of the SMT160 driver.

## 🏗️ Architectural Overview

### System Module Relationships
Describes how the different crates and modules interact to form the full driver stack.

```mermaid
graph TD
    subgraph "Application"
        App[User Firmware]
        RTIC[RTIC Task]
    end

    subgraph "Driver Core (no_std)"
        Lib[lib.rs: Smt160Driver]
        Math[math.rs: Fixed-Point Core]
        Types[types.rs: Typestate]
    end

    subgraph "Hardware (HAL)"
        Trait[Smt160Hal Trait]
        HW[Timer/DMA Peripheral]
    end

    App --> Lib
    RTIC --> Lib
    Lib --> Trait
    Lib --> Math
    Lib --> Types
    Trait --> HW
```

---

## 📡 Signal Processing Flow

### PWM Edge Capture to Temperature Output
Illustrates the sequence of events from a physical edge on the pin to a filtered temperature reading.

```mermaid
sequenceDiagram
    participant S as SMT160 Sensor
    participant T as Timer HW
    participant C as Smt160Hal
    participant D as Smt160Driver
    participant M as SignalDecoder
    participant A as Application

    S->>T: Rising Edge
    T->>C: Capture period & high
    C->>D: read_raw() -> CapturedEdge
    D->>M: decode(period, high)
    M-->>D: Raw Temperature
    D->>M: apply_adaptive_filter()
    M-->>D: Filtered Temperature
    D-->>A: Result<I32F32, Error>
```

---

## 🔄 State Machine Logic

### Decoder Internal Transitions
Shows how the decoder handles incomplete cycles and potential errors.

```mermaid
stateDiagram-v2
    [*] --> Idle
    Idle --> WaitingForFall: Rising Edge (T1)
    WaitingForFall --> WaitingForRise: Falling Edge (T2)
    WaitingForFall --> Error: Timeout / Signal Loss
    WaitingForRise --> Processing: Second Rising Edge (T3)
    WaitingForRise --> Error: Invalid Duty Cycle
    Processing --> Idle: Success (Update Filter)
    Processing --> Error: Frequency Drift
    Error --> Idle: Reset State
```

---

## 📈 Adaptive Filtering Behavior

### Filter Alpha Selection Logic
Visualizes how the driver balances responsiveness with noise rejection.

```mermaid
graph LR
    Start{New Sample}
    Diff{Deviation > 5°C?}
    Startup{Samples < 16?}
    
    Start --> Diff
    Diff -- Yes --> Fast[Alpha = 0.8: Fast Track]
    Diff -- No --> Startup
    Startup -- Yes --> Fast
    Startup -- No --> Slow[Alpha = 0.1: Noise Rejection]
    
    Fast --> Output
    Slow --> Output
```
