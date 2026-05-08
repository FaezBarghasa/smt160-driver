//! Typestate definitions for the SMT160 driver.
//!
//! This module implements the Typestate pattern to enforce hardware safety at compile time.
//! By encoding the device state into the Rust type system, we prevent the user from
//! attempting to read from the sensor before the hardware peripherals (Timers, DMA, Clocks)
//! have been properly initialized and validated.

/// Zero-sized marker struct representing an uninitialized SMT160 driver.
///
/// In this state, the driver has no ownership of hardware peripherals and 
/// cannot perform any measurements. This is the starting point for the driver lifecycle.
pub struct Uninitialized;

/// Zero-sized marker struct representing a fully initialized and validated SMT160 driver.
///
/// Transitions to this state only occur after the clock frequency has been verified 
/// to meet the 0.05°C precision requirements and the DMA/Timer subsystems are active.
pub struct Ready;
