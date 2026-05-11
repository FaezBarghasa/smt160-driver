use crate::error::Smt160Error;
use crate::hal::{Smt160Hal, CapturedEdge};
use stm32f1xx_hal::pac;
use stm32f1xx_hal::rcc::Clocks;
use portable_atomic::AtomicU32;
use core::sync::atomic::Ordering;

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
/// This struct ensures that the raw [u32; N] buffer is correctly aligned and 
/// can be safely viewed as a sequence of `CapturedEdge` records.
#[repr(C, align(4))]
pub struct Smt160DmaBuffer<const N: usize> {
    raw: [u32; N],
}

impl<const N: usize> Smt160DmaBuffer<N> {
    /// Creates a new, zero-initialized DMA buffer.
    pub const fn new() -> Self {
        Self { raw: [0; N] }
    }

    /// Returns a raw pointer to the start of the buffer for DMA configuration.
    pub fn as_mut_ptr(&mut self) -> *mut u32 {
        self.raw.as_mut_ptr()
    }

    /// Returns a reference to the captured edge at the specified index.
    /// 
    /// Each burst capture transfers 2 words: CCR1 (Period) and CCR2 (High Time).
    #[inline(always)]
    pub fn get_edge(&self, index: usize) -> CapturedEdge {
        let period_val = self.raw[index * 2];
        let high_val = self.raw[index * 2 + 1];
        CapturedEdge {
            period_ticks: (period_val & 0xFFFF) as u64,
            high_ticks: (high_val & 0xFFFF) as u64,
        }
    }
}

use embassy_sync::waitqueue::AtomicWaker;

/// Optimized HAL for STM32F103C8T6 (BluePill).
/// 
/// Recommended Pin Mappings for BluePill:
/// - TIM2: PA0 (CH1), PA1 (CH2)
/// - TIM3: PA6 (CH1), PA7 (CH2)
/// - TIM4: PB6 (CH1), PB7 (CH2)
pub struct Stm32F1DmaHal<'a, TIM, DMA, const N: usize> 
where 
    TIM: Smt160TimerInstance,
    DMA: Smt160DmaChannel,
{
    timer: TIM,
    dma: DMA,
    buffer: &'a mut Smt160DmaBuffer<N>,
    waker: AtomicWaker,
    timer_channel: u8,
    buffer_len: u16,
    overflow_count: AtomicU32,
}

impl<'a, TIM, DMA, const N: usize> Stm32F1DmaHal<'a, TIM, DMA, N> 
where 
    TIM: Smt160TimerInstance,
    DMA: Smt160DmaChannel,
{
    /// Creates a new STM32F1 DMA adapter for a specific timer channel (1 or 3).
    pub fn new(timer: TIM, dma: DMA, buffer: &'a mut Smt160DmaBuffer<N>, timer_channel: u8, buffer_len: u16) -> Self {
        Self { 
            timer, 
            dma, 
            buffer,
            waker: AtomicWaker::new(),
            timer_channel,
            buffer_len,
            overflow_count: AtomicU32::new(0),
        }
    }

    /// Called from the Timer Update (Overflow) interrupt.
    pub fn handle_timer_overflow(&self) {
        self.overflow_count.fetch_add(1, Ordering::Relaxed);
    }

    /// Returns the current state of the timer (SR, CCER) to ensure capture is actually happening on the pins.
    pub fn check_timer_state(&self) -> (u32, u32) {
        self.timer.check_timer_state()
    }
}

impl<'a, TIM, DMA, const N: usize> Smt160Hal for Stm32F1DmaHal<'a, TIM, DMA, N>
where 
    TIM: Smt160TimerInstance,
    DMA: Smt160DmaChannel,
{
    fn setup(&mut self, freq: u32) -> Result<(), Smt160Error> {
        // 1. Full reset: zero ALL timer registers to known defaults
        self.timer.reset_hardware();

        // 2. Set prescaler (PSC is a preload register, loaded on next update event)
        let sysclk = 72_000_000; // STM32F1 APB1 timer clock (72MHz when APB1 prescaler > 1)
        let psc = (sysclk / freq).saturating_sub(1) as u16;
        self.timer.set_prescaler(psc);

        // 3. Set ARR to maximum so the counter has full 16-bit range
        self.timer.set_arr(0xFFFF);

        // 4. Configure PWM input mode (CCMR, CCER, SMCR)
        self.timer.setup_pwm_input(self.timer_channel);

        // 5. Configure DMA burst (DCR, DIER)
        self.timer.setup_dma_burst(self.timer_channel);

        // 6. Configure DMA channel
        unsafe {
            let dmar_ptr = self.timer.dmar_address();
            #[cfg(feature = "defmt")]
            defmt::info!("DMA Debug: Pointing to DMAR at {:#X}", dmar_ptr);
            
            self.dma.setup_circular_capture(
                dmar_ptr,
                self.buffer.as_mut_ptr(),
                self.buffer_len
            );
        }

        // 7. Generate update event to load PSC and ARR into active registers
        self.timer.generate_update();

        // 8. Clear all SR flags that were set by the UG event
        self.timer.clear_status();

        // 9. Enable the timer
        self.timer.enable();

        Ok(())
    }

    #[inline(always)]
    fn is_new_data_available(&self) -> bool {
        let ht = self.dma.is_half_transfer();
        let tc = self.dma.is_transfer_complete();
        if ht || tc {
            #[cfg(feature = "defmt")]
            defmt::info!("DMA Data: HT={}, TC={}", ht, tc);
        }
        ht || tc
    }

    #[inline(always)]
    fn read_raw(&self) -> CapturedEdge {
        // Read CNDTR to find the most recent sample
        let cndtr = self.dma.get_cndtr();
        let elements_written = self.buffer_len - cndtr as u16;
        
        // Each edge consists of a 2-word burst. 
        // We only want to read the last fully completed 2-word pair.
        let full_edges_written = elements_written / 2;
        let edge_idx = if full_edges_written == 0 {
            (self.buffer_len / 2).saturating_sub(1)
        } else {
            full_edges_written.saturating_sub(1)
        };
        
        let edge = self.buffer.get_edge(edge_idx as usize); 
        self.dma.clear_interrupt_flags();
        edge
    }

    fn wait_for_new_data(&mut self) -> impl core::future::Future<Output = Result<(), Smt160Error>> {
        core::future::poll_fn(|cx| {
            self.waker.register(cx.waker());
            if self.is_new_data_available() {
                core::task::Poll::Ready(Ok(()))
            } else {
                core::task::Poll::Pending
            }
        })
    }

    fn notify(&self) {
        self.waker.wake();
    }
}

impl<'a, TIM, DMA, const N: usize> Drop for Stm32F1DmaHal<'a, TIM, DMA, N>
where
    TIM: Smt160TimerInstance,
    DMA: Smt160DmaChannel,
{
    fn drop(&mut self) {
        #[cfg(feature = "defmt")]
        defmt::info!("Stm32F1DmaHal DROPPED - Disabling Hardware");
        self.dma.disable();
        self.timer.reset_hardware();
    }
}

/// Trait representing a Timer capable of advanced PWM Input + DMA Burst.
pub trait Smt160TimerInstance {
    /// Configures the timer for PWM Input mode on the specified channel pair.
    fn setup_pwm_input(&self, channel: u8);
    
    /// Configures the DMA Burst (DMAR) to fetch capture registers for the given channel.
    fn setup_dma_burst(&self, channel: u8);
    
    /// Returns the physical address of the Timer's DMA Burst (DMAR) register.
    fn dmar_address(&self) -> u32;

    /// Returns the current state of the timer (SR, CCER). Useful for debugging.
    fn check_timer_state(&self) -> (u32, u32);

    /// Enables the timer (sets CEN).
    fn enable(&self);

    /// Generates an update event to load shadow registers (PSC, ARR).
    fn generate_update(&self);

    /// Clears all status register flags.
    fn clear_status(&self);

    /// Performs a full reset of all timer registers to their default state.
    fn reset_hardware(&self);

    /// Sets the timer prescaler.
    fn set_prescaler(&self, psc: u16);

    /// Sets the auto-reload register.
    fn set_arr(&self, arr: u16);
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
    
    /// Returns the current value of the CNDTR register.
    fn get_cndtr(&self) -> u32;
}

// ============================================================================
// TIMER MACRO
// ============================================================================

macro_rules! impl_smt160_timer {
    ($TIMX:ident) => {
        impl Smt160TimerInstance for pac::$TIMX {
            fn setup_pwm_input(&self, channel: u8) {
                match channel {
                    1 => {
                        // CCMR1 in input capture mode:
                        //   CC1S = 01 (IC1 mapped to TI1)
                        //   CC2S = 10 (IC2 mapped to TI1)
                        //   IC1F = 0000 (no input filter)
                        //   IC2F = 0000 (no input filter)
                        //   IC1PSC = 00 (capture on every event)
                        //   IC2PSC = 00 (capture on every event)
                        // Using write (NOT modify) to guarantee a clean register state.
                        self.ccmr1_input().write(|w| w.cc1s().ti1().cc2s().ti1());

                        // CCER: CC1 rising edge, CC2 falling edge, both enabled
                        // Using write to guarantee no stale bits from other channels.
                        self.ccer.write(|w| {
                            w.cc1p().clear_bit()  // CC1: non-inverted (rising edge)
                             .cc1e().set_bit()     // CC1: enable
                             .cc2p().set_bit()     // CC2: inverted (falling edge)
                             .cc2e().set_bit()     // CC2: enable
                        });

                        // SMCR: Slave mode reset on TI1FP1 rising edge
                        self.smcr.write(|w| w.ts().ti1fp1().sms().reset_mode());
                    }
                    3 => {
                        // CCMR2 in input capture mode:
                        //   CC3S = 01 (IC3 mapped to TI3)
                        //   CC4S = 10 (IC4 mapped to TI3)
                        self.ccmr2_input().write(|w| w.cc3s().ti3().cc4s().ti3());

                        self.ccer.write(|w| {
                            w.cc3p().clear_bit()
                             .cc3e().set_bit()
                             .cc4p().set_bit()
                             .cc4e().set_bit()
                        });

                        // Note: STM32F1 Slave Mode Reset only supports TI1FP1 and TI2FP2.
                        self.smcr.write(|w| w.ts().ti1fp1().sms().reset_mode());
                    }
                    _ => panic!("SMT160 driver only supports channels 1 and 3 on STM32F1"),
                }
            }

            fn setup_dma_burst(&self, channel: u8) {
                match channel {
                    1 => {
                        // DCR: DBA=13 (CCR1 offset), DBL=1 (2 transfers: CCR1 + CCR2)
                        self.dcr.write(|w| unsafe { w.dba().bits(13).dbl().bits(1) });
                        // DIER: enable CC1 DMA request (triggers burst on each capture)
                        self.dier.write(|w| w.cc1de().set_bit());
                    }
                    3 => {
                        // DCR: DBA=15 (CCR3 offset), DBL=1 (2 transfers: CCR3 + CCR4)
                        self.dcr.write(|w| unsafe { w.dba().bits(15).dbl().bits(1) });
                        // DIER: enable CC3 DMA request
                        self.dier.write(|w| w.cc3de().set_bit());
                    }
                    _ => panic!("SMT160 driver only supports channels 1 and 3 on STM32F1"),
                }
            }

            fn dmar_address(&self) -> u32 {
                self.dmar.as_ptr() as u32
            }

            fn check_timer_state(&self) -> (u32, u32) {
                (self.sr.read().bits(), self.ccer.read().bits())
            }

            fn enable(&self) {
                self.cr1.write(|w| w.cen().set_bit());
            }

            fn generate_update(&self) {
                self.egr.write(|w| w.ug().set_bit());
            }

            fn clear_status(&self) {
                self.sr.write(|w| unsafe { w.bits(0) });
            }

            fn reset_hardware(&self) {
                // Full reset: write all registers to their default values.
                // This guarantees no stale configuration from a previous run.
                self.cr1.write(|w| unsafe { w.bits(0) });
                self.cr2.write(|w| unsafe { w.bits(0) });
                self.smcr.write(|w| unsafe { w.bits(0) });
                self.dier.write(|w| unsafe { w.bits(0) });
                self.sr.write(|w| unsafe { w.bits(0) });
                self.ccmr1_output().write(|w| unsafe { w.bits(0) });
                self.ccmr2_output().write(|w| unsafe { w.bits(0) });
                self.ccer.write(|w| unsafe { w.bits(0) });
                self.cnt.reset();
                self.psc.write(|w| w.psc().bits(0));
                self.arr.write(|w| w.arr().bits(0xFFFF));
                self.dcr.write(|w| unsafe { w.bits(0) });
            }

            fn set_prescaler(&self, psc: u16) {
                self.psc.write(|w| w.psc().bits(psc));
            }

            fn set_arr(&self, arr: u16) {
                self.arr.write(|w| w.arr().bits(arr));
            }
        }
    };
}

impl_smt160_timer!(TIM1);
impl_smt160_timer!(TIM2);
impl_smt160_timer!(TIM3);
impl_smt160_timer!(TIM4);
impl_smt160_timer!(TIM5);
impl_smt160_timer!(TIM8);

// ============================================================================
// DMA MACRO
// ============================================================================

macro_rules! impl_smt160_dma {
    ($HAL_MOD:ident, $PAC_PERIPH:ident, $($CH:ident, $field:ident, $offset:expr),+) => {
        $(
            impl Smt160DmaChannel for stm32f1xx_hal::dma::$HAL_MOD::$CH {
                unsafe fn setup_circular_capture(&self, peripheral_addr: u32, memory_addr: *mut u32, len: u16) {
                    let ch_base = 0x40020000 + 0x08 + ($offset * 0x14);
                    let cr_ptr = ch_base as *mut u32;
                    let ndtr_ptr = (ch_base + 0x04) as *mut u32;
                    let par_ptr = (ch_base + 0x08) as *mut u32;
                    let mar_ptr = (ch_base + 0x0C) as *mut u32;

                    // SAFETY: Direct register access for DMA channel configuration. 
                    // This is safe because the HAL has exclusive ownership of the channel.
                    unsafe {
                        // 1. Disable channel and wait for it to stop
                        core::ptr::write_volatile(cr_ptr, core::ptr::read_volatile(cr_ptr) & !1);
                        while (core::ptr::read_volatile(cr_ptr) & 1) != 0 {}

                        // 2. Load addresses and length
                        core::ptr::write_volatile(par_ptr, peripheral_addr);
                        core::ptr::write_volatile(mar_ptr, memory_addr as u32);
                        core::ptr::write_volatile(ndtr_ptr, len as u32);

                        // 3. Configure and Enable
                        // 0xAAF: MSIZE=32bit (10), PSIZE=32bit (10), MINC (1), CIRC (1), TEIE, HTIE, TCIE, EN
                        core::ptr::write_volatile(cr_ptr, 0xAAF);
                    }
                }

                fn clear_interrupt_flags(&self) {
                    let dma_isr_base = 0x40020000 + 0x04; // DMA1_IFCR
                    // SAFETY: Clearing interrupt flags is safe as it only affects status bits.
                    unsafe {
                        core::ptr::write_volatile(dma_isr_base as *mut u32, 0xF << ($offset * 4));
                    }
                }
                
                fn is_half_transfer(&self) -> bool {
                    // SAFETY: Reading DMA ISR status flags is safe.
                    let dma_isr = unsafe { (*pac::$PAC_PERIPH::ptr()).isr.read().bits() };
                    (dma_isr & (1 << (($offset * 4) + 2))) != 0 // HTIFx is bit 2 of the 4-bit block
                }
                
                fn is_transfer_complete(&self) -> bool {
                    // SAFETY: Reading DMA ISR status flags is safe.
                    let dma_isr = unsafe { (*pac::$PAC_PERIPH::ptr()).isr.read().bits() };
                    (dma_isr & (1 << (($offset * 4) + 1))) != 0 // TCIFx is bit 1 of the 4-bit block
                }

                fn disable(&self) {
                    let ch_base = 0x40020000 + 0x08 + ($offset * 0x14);
                    // SAFETY: Disabling the DMA channel is safe during drop or shutdown.
                    unsafe {
                        core::ptr::write_volatile(ch_base as *mut u32, core::ptr::read_volatile(ch_base as *mut u32) & !1);
                    }
                }
                
                fn get_cndtr(&self) -> u32 {
                    let ch_base = 0x40020000 + 0x08 + ($offset * 0x14);
                    unsafe { core::ptr::read_volatile((ch_base + 0x04) as *const u32) }
                }
            }
        )+
    }
}

impl_smt160_dma!(dma1, DMA1, C1, ch1, 0, C2, ch2, 1, C3, ch3, 2, C4, ch4, 3, C5, ch5, 4, C6, ch6, 5, C7, ch7, 6);

// Support DMA2 for High-density devices (TIM5, TIM8, etc.)
#[cfg(feature = "high")]
impl_smt160_dma!(dma2, DMA2, C1, ch1, 0, C2, ch2, 1, C3, ch3, 2, C4, ch4, 3, C5, ch5, 4);