#![no_std]

//! # SMT160 High-Precision Industrial Driver
//!
//! This driver uses hardware-level DMA Burst and Timer Reset Mode to achieve 
//! absolute zero-jitter capture of SMT160 temperature sensor signals.
//!
//! ## Features
//! - **Typestate Safety:** Compile-time prevention of uninitialized access.
//! - **DMA Burst:** Zero CPU overhead during signal capture.
//! - **Fixed-Point Math:** Deterministic 0.05°C precision on Cortex-M3.

pub mod error;
pub mod types;
pub mod math;
pub mod telemetry;
pub mod hal;

pub use error::Smt160Error;
pub use types::{Uninitialized, Ready};
pub use math::SignalDecoder;
pub use telemetry::Smt160Status;

use fixed::types::I32F32;
use core::marker::PhantomData;
use crate::hal::{Smt160TimerInstance, Smt160DmaChannel};

/// The industrial-grade SMT160 driver using DMA Burst.
pub struct Smt160<State, TIM, DMA> {
    _state: PhantomData<State>,
    pub(crate) timer: TIM,
    pub(crate) dma: DMA,
    buffer: &'static mut [u32; 4],
    last_period: u32,
    watchdog_ticks: u32,
    last_dma_count: u16,
    pub status: Smt160Status,
}

impl<TIM, DMA> Smt160<Uninitialized, TIM, DMA> 
where 
    TIM: Smt160TimerInstance,
    DMA: Smt160DmaChannel,
{
    /// Creates a new uninitialized driver instance.
    pub fn new(timer: TIM, dma: DMA, buffer: &'static mut [u32; 4]) -> Self {
        Self {
            _state: PhantomData,
            timer,
            dma,
            buffer,
            last_period: 0,
            watchdog_ticks: 0,
            last_dma_count: 0,
            status: Smt160Status::empty(),
        }
    }

    /// Initializes hardware and transitions to the `Ready` state.
    pub fn init(self, clocks: &stm32f1xx_hal::rcc::Clocks) -> Result<Smt160<Ready, TIM, DMA>, Smt160Error> {
        crate::hal::validate_clocks(clocks)?;
        
        self.timer.reset_hardware();
        self.timer.setup_pwm_input();
        self.timer.setup_dma_burst();

        // Safety: Buffer is 'static and correctly sized for circular capture (2 signals * 2 halves = 4)
        unsafe {
            self.dma.setup_circular_capture(
                self.timer.dmar_address(),
                self.buffer.as_mut_ptr(),
                4
            );
        }

        Ok(Smt160 {
            _state: PhantomData,
            timer: self.timer,
            dma: self.dma,
            buffer: self.buffer,
            last_period: 0,
            watchdog_ticks: 0,
            last_dma_count: 0,
            status: Smt160Status::empty(),
        })
    }
}

impl<TIM, DMA> Smt160<Ready, TIM, DMA> 
where 
    TIM: Smt160TimerInstance,
    DMA: Smt160DmaChannel,
{
    /// Polls the DMA for new data and performs jitter diagnostics.
    /// 
    /// This method parses the DMA ISR flags to determine which half of the 
    /// circular buffer is safe to read, ensuring zero-copy and zero-jitter.
    pub fn poll_dma(&mut self) -> Option<I32F32> {
        let mut sample = None;

        if self.dma.is_half_transfer() {
            // DMA finished first half [0..2]. It is now writing [2..4].
            // CCR1 (Period) is at index 0, CCR2 (Active) is at index 1.
            sample = Some((self.buffer[0], self.buffer[1]));
            self.dma.clear_interrupt_flags();
        } else if self.dma.is_transfer_complete() {
            // DMA finished second half [2..4]. It is now wrapping back to [0..2].
            // CCR1 (Period) is at index 2, CCR2 (Active) is at index 3.
            sample = Some((self.buffer[2], self.buffer[3]));
            self.dma.clear_interrupt_flags();
        }

        if let Some((period, active)) = sample {
            // Jitter Detection: If period changes by > 0.5% (1/200)
            if self.last_period > 0 {
                let delta = if period > self.last_period { period - self.last_period } else { self.last_period - period };
                if delta > (self.last_period / 200) {
                    self.status.insert(Smt160Status::JITTER_DETECTED);
                } else {
                    self.status.remove(Smt160Status::JITTER_DETECTED);
                }
            }
            self.last_period = period;

            match SignalDecoder::decode(period, active) {
                Ok(raw) => {
                    self.status.remove(Smt160Status::OUT_OF_BOUNDS);
                    Some(SignalDecoder::apply_nlc(raw))
                }
                Err(_) => {
                    self.status.insert(Smt160Status::OUT_OF_BOUNDS);
                    None
                }
            }
        } else {
            None
        }
    }

    /// Watchdog called every 1ms to detect sensor flatline or ESD freeze.
    /// 
    /// `current_dma_count` is retrieved from the CNDTR register.
    pub fn tick_watchdog(&mut self, current_dma_count: u16) -> Result<(), Smt160Error> {
        if current_dma_count == self.last_dma_count {
            self.watchdog_ticks += 1;
        } else {
            self.watchdog_ticks = 0;
            self.status.remove(Smt160Status::SENSOR_TIMEOUT);
        }
        self.last_dma_count = current_dma_count;

        // If no DMA transaction occurs for 5ms (sensor max period is ~1ms)
        if self.watchdog_ticks >= 5 {
            self.status.insert(Smt160Status::SENSOR_TIMEOUT);
            return Err(Smt160Error::SensorTimeout);
        }
        Ok(())
    }

    /// Autonomous Hardware Recovery.
    /// Disables peripherals, clears flags, and re-initializes to recover from ESD hardware freezes.
    pub fn hard_reset(&mut self) {
        self.dma.disable();
        self.timer.reset_hardware();
        self.status = Smt160Status::empty();
        self.watchdog_ticks = 0;
        self.last_period = 0;
        
        // Re-init
        self.timer.setup_pwm_input();
        self.timer.setup_dma_burst();
        unsafe {
            self.dma.setup_circular_capture(
                self.timer.dmar_address(),
                self.buffer.as_mut_ptr(),
                4
            );
        }
    }
}

