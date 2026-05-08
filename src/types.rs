//! Typestate definitions for the SMT160 driver.
//!
//! This module implements the Typestate pattern to enforce hardware safety at compile time.
//! By encoding the device state into the Rust type system, we prevent the user from
//! attempting to read from the sensor before the hardware peripherals (Timers, DMA, Clocks)
//! have been properly initialized and validated.

use core::marker::PhantomData;

/// Zero-sized marker struct representing an uninitialized SMT160 driver.
///
/// In this state, the driver has no ownership of hardware peripherals and 
/// cannot perform any measurements.
pub struct Uninitialized;

/// Zero-sized marker struct representing a fully initialized and validated SMT160 driver.
///
/// Transitions to this state only occur after the clock frequency has been verified 
/// to meet the 0.05°C precision requirements and the DMA/Timer subsystems are active.
pub struct Ready;

/// The main SMT160 driver structure.
///
/// The `State` parameter is used to track the initialization status of the hardware.
/// 
/// # Typestate Benefits:
/// - **Zero-Cost:** State transitions are checked at compile time and have no runtime overhead.
/// - **Safety:** Methods like `read_temperature()` are only implemented for `Smt160<Ready>`.
pub struct Smt160<State> {
    _state: PhantomData<State>,
    // Hardware peripheral ownership will be added in Phase 3/4
}

impl Smt160<Uninitialized> {
    /// Creates a new, uninitialized instance of the SMT160 driver.
    pub const fn new() -> Self {
        Self {
            _state: PhantomData,
        }
    }
}
