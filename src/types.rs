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

/// Zero-sized marker struct representing an actively polling SMT160 driver.
pub struct Running;

use fixed::types::I32F32;

/// Edge trigger direction for threshold notifications.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum TriggerEdge {
    Rising,
    Falling,
    Both,
}

/// Trait-based observer for RTIC 2.1 integration.
/// 
/// Implementing this trait allows external tasks to receive notifications 
/// on critical sensor events without polling the status register.
pub trait Smt160Observer: Send + Sync {
    /// Called when a new temperature sample is processed.
    fn on_threshold_crossed(&self, temp: I32F32);
    
    /// Called when the sensor signal is lost (timeout).
    fn on_signal_lost(&self);
    
    /// Called when a hardware error (e.g., DMA failure) is detected.
    fn on_hardware_error(&self);
}

impl Smt160Observer for () {
    #[inline(always)]
    fn on_threshold_crossed(&self, _temp: I32F32) {}
    #[inline(always)]
    fn on_signal_lost(&self) {}
    #[inline(always)]
    fn on_hardware_error(&self) {}
}
