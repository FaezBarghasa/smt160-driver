use crate::error::Smt160Error;
use crate::hal::{Smt160Hal, CapturedEdge};
use stm32f1xx_hal::pac;
use stm32f1xx_hal::rcc::Clocks;

/// Validates that the APB1 clock is running at least at 8 MHz.
/// This is the absolute minimum resolution (125ns/tick) required to
/// guarantee the 0.05°C accuracy specification. 72 MHz (13.8ns/tick) is preferred.
pub fn validate_clocks(clocks: &Clocks) -> Result<(), Smt160Error> {
    let pclk1_hz = clocks.pclk1().to_Hz();
    if pclk1_hz < 8_000_000 {
        Err(Smt160Error::ClockTooSlow)
    } else {
        Ok(())
    }
}

/// Trivial wrapper around the DMA buffer to allow zero-copy access from the driver.
///
/// This struct ensures that the raw [u32; 4] buffer is correctly aligned and 
/// can be safely viewed as a sequence of `CapturedEdge` records.
#[repr(C, align(4))]
pub struct Smt160DmaBuffer {
    raw: [u32; 4],
}

impl Smt160DmaBuffer {
    /// Creates a new, zero-initialized DMA buffer.
    pub const fn new() -> Self {
        Self { raw: [0; 4] }
    }

    /// Returns a raw pointer to the start of the buffer for DMA configuration.
    pub fn as_mut_ptr(&mut self) -> *mut u32 {
        self.raw.as_mut_ptr()
    }

    /// Returns a reference to the captured edge in the specified half of the buffer.
    ///
    /// # Safety
    /// This is safe because `CapturedEdge` is `repr(C)` and matches the layout 
    /// of two consecutive `u32` words.
    #[inline(always)]
    pub fn get_edge(&self, half: bool) -> &CapturedEdge {
        unsafe {
            if half {
                &*(self.raw.as_ptr() as *const CapturedEdge)
            } else {
                &*(self.raw.as_ptr().add(2) as *const CapturedEdge)
            }
        }
    }
}

use embassy_sync::waitqueue::AtomicWaker;

/// STM32F1-specific implementation of the SMT160 HAL using DMA Burst and Timer Slave-Reset.
pub struct Stm32F1DmaHal<TIM, DMA> {
    timer: TIM,
    dma: DMA,
    buffer: &'static mut Smt160DmaBuffer,
    waker: AtomicWaker,
    timer_channel: u8,
}

impl<TIM, DMA> Stm32F1DmaHal<TIM, DMA> 
where 
    TIM: Smt160TimerInstance,
    DMA: Smt160DmaChannel,
{
    /// Creates a new STM32F1 DMA adapter for a specific timer channel (1 or 3).
    pub fn new(timer: TIM, dma: DMA, buffer: &'static mut Smt160DmaBuffer, timer_channel: u8) -> Self {
        Self { 
            timer, 
            dma, 
            buffer,
            waker: AtomicWaker::new(),
            timer_channel,
        }
    }
}

impl<TIM, DMA> Smt160Hal for Stm32F1DmaHal<TIM, DMA>
where 
    TIM: Smt160TimerInstance,
    DMA: Smt160DmaChannel,
{
    fn setup(&mut self, _freq: u32) -> Result<(), Smt160Error> {
        self.timer.reset_hardware();
        self.timer.setup_pwm_input(self.timer_channel);
        self.timer.setup_dma_burst(self.timer_channel);

        unsafe {
            self.dma.setup_circular_capture(
                self.timer.dmar_address(),
                self.buffer.as_mut_ptr(),
                4
            );
        }
        Ok(())
    }

    #[inline(always)]
    fn is_new_data_available(&self) -> bool {
        self.dma.is_half_transfer() || self.dma.is_transfer_complete()
    }

    #[inline(always)]
    fn read_raw(&self) -> CapturedEdge {
        let is_ht = self.dma.is_half_transfer();
        let edge = self.buffer.get_edge(is_ht);
        self.dma.clear_interrupt_flags();
        *edge
    }

    async fn wait_for_new_data(&mut self) -> Result<(), Smt160Error> {
        core::future::poll_fn(|cx| {
            self.waker.register(cx.waker());
            if self.is_new_data_available() {
                core::task::Poll::Ready(Ok(()))
            } else {
                core::task::Poll::Pending
            }
        }).await
    }

    fn notify(&self) {
        self.waker.wake();
    }
}

/// Trait representing a Timer capable of advanced PWM Input + DMA Burst.
pub trait Smt160TimerInstance {
    /// Configures the timer for PWM Input mode on the specified channel pair.
    /// Channel pair 1: TI1/CC1/CC2, Channel pair 2: TI2/CC2/CC1 (not common), 
    /// Channel pair 3: TI3/CC3/CC4, Channel pair 4: TI4/CC4/CC3.
    fn setup_pwm_input(&self, channel: u8);
    
    /// Configures the DMA Burst (DMAR) to fetch capture registers for the given channel.
    fn setup_dma_burst(&self, channel: u8);
    
    /// Returns the physical address of the Timer's DMA Burst (DMAR) register.
    fn dmar_address(&self) -> u32;
    
    /// Disables and resets the hardware.
    fn reset_hardware(&self);
}

/// Trait representing a DMA Channel mapped to a Timer event.
pub trait Smt160DmaChannel {
    /// Configures the DMA channel for circular transfers.
    /// 
    /// # Safety
    /// `memory_addr` must point to a valid, pinned buffer.
    unsafe fn setup_circular_capture(&self, peripheral_addr: u32, memory_addr: *mut u32, len: u16);
    
    /// Clears all interrupt flags.
    fn clear_interrupt_flags(&self);
    
    /// Checks if the Half Transfer flag is set.
    fn is_half_transfer(&self) -> bool;
    
    /// Checks if the Transfer Complete flag is set.
    fn is_transfer_complete(&self) -> bool;
    
    /// Disables the DMA channel.
    fn disable(&self);
}

// ... rest of the file remains same with macros

// ============================================================================
// TIMER MACRO
// ============================================================================

macro_rules! impl_smt160_timer {
    ($TIMX:ident) => {
        impl Smt160TimerInstance for pac::$TIMX {
            fn setup_pwm_input(&self, channel: u8) {
                match channel {
                    1 => {
                        self.ccmr1_input().modify(|_, w| w.cc1s().ti1().cc2s().ti1());
                        self.ccer.modify(|_, w| w.cc1p().clear_bit().cc2p().set_bit().cc1e().set_bit().cc2e().set_bit());
                        self.smcr.modify(|_, w| w.ts().ti1fp1().sms().reset_mode());
                    }
                    3 => {
                        self.ccmr2_input().modify(|_, w| w.cc3s().ti3().cc4s().ti3());
                        self.ccer.modify(|_, w| w.cc3p().clear_bit().cc4p().set_bit().cc3e().set_bit().cc4e().set_bit());
                        // Note: STM32F1 Slave Mode Reset only supports TI1FP1 and TI2FP2.
                        // For CH3/CH4, we still use TI1FP1 as the reset source if they are synchronized,
                        // or this driver configuration may be invalid for independent CH3/CH4 sensors.
                        self.smcr.modify(|_, w| w.ts().ti1fp1().sms().reset_mode());
                    }
                    _ => panic!("SMT160 driver only supports channels 1 and 3 on STM32F1"),
                }
                self.cr1.modify(|_, w| w.cen().set_bit());
            }

            fn setup_dma_burst(&self, channel: u8) {
                match channel {
                    1 => {
                        self.dcr.modify(|_, w| unsafe { w.dba().bits(13).dbl().bits(1) });
                        self.dier.modify(|_, w| w.cc1de().set_bit());
                    }
                    3 => {
                        self.dcr.modify(|_, w| unsafe { w.dba().bits(15).dbl().bits(1) });
                        self.dier.modify(|_, w| w.cc3de().set_bit());
                    }
                    _ => panic!("SMT160 driver only supports channels 1 and 3 on STM32F1"),
                }
            }

            fn dmar_address(&self) -> u32 {
                self.dmar.as_ptr() as u32
            }

            fn reset_hardware(&self) {
                self.cr1.modify(|_, w| w.cen().clear_bit());
                self.dier.modify(|_, w| w.cc1de().clear_bit().cc2de().clear_bit().cc3de().clear_bit().cc4de().clear_bit());
                self.sr.write(|w| unsafe { w.bits(0) });
            }
        }
    };
}

impl_smt160_timer!(TIM2);
impl_smt160_timer!(TIM3);
impl_smt160_timer!(TIM4);

// ============================================================================
// DMA MACRO
// ============================================================================

macro_rules! impl_smt160_dma {
    ($($CH:ident, $field:ident, $offset:expr),+) => {
        $(
            impl Smt160DmaChannel for stm32f1xx_hal::dma::dma1::$CH {
                unsafe fn setup_circular_capture(&self, peripheral_addr: u32, memory_addr: *mut u32, len: u16) {
                    let dma1 = unsafe { &*pac::DMA1::ptr() };
                    let ch = &dma1.$field;

                    // Disable before configuration
                    ch.cr.modify(|_, w| w.en().clear_bit());

                    ch.par.write(|w| unsafe { w.pa().bits(peripheral_addr) });
                    ch.mar.write(|w| unsafe { w.ma().bits(memory_addr as u32) });
                    ch.ndtr.write(|w| unsafe { w.ndt().bits(len) });

                    // CR: 32-bit MSIZE/PSIZE, MINC, CIRC, HTIE, TCIE, EN
                    ch.cr.modify(|_, w| unsafe {
                        w.msize().bits(0b10);
                        w.psize().bits(0b10);
                        w.minc().set_bit();
                        w.circ().set_bit();
                        w.htie().set_bit();
                        w.tcie().set_bit();
                        w.en().set_bit()
                    });
                }

                fn clear_interrupt_flags(&self) {
                    let dma1 = unsafe { &*pac::DMA1::ptr() };
                    dma1.ifcr.write(|w| unsafe { w.bits(0xF << ($offset * 4)) });
                }

                fn is_half_transfer(&self) -> bool {
                    let dma1 = unsafe { &*pac::DMA1::ptr() };
                    (dma1.isr.read().bits() >> ($offset * 4 + 2)) & 1 != 0
                }

                fn is_transfer_complete(&self) -> bool {
                    let dma1 = unsafe { &*pac::DMA1::ptr() };
                    (dma1.isr.read().bits() >> ($offset * 4 + 1)) & 1 != 0
                }

                fn disable(&self) {
                    let dma1 = unsafe { &*pac::DMA1::ptr() };
                    dma1.$field.cr.modify(|_, w| w.en().clear_bit());
                }
            }
        )+
    }
}

impl_smt160_dma!(C1, ch1, 0, C4, ch4, 3, C5, ch5, 4, C6, ch6, 5);
