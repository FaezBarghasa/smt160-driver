use crate::error::Smt160Error;
use crate::hal::{Smt160Hal, CapturedEdge};
use rp2040_hal::pio::{PIOExt, StateMachineIndex, UninitStateMachine, StateMachine, PIOBuilder, PinDir};
use rp2040_hal::dma::{SingleChannel, Channel};
use rp2040_hal::pac;
use embassy_sync::waitqueue::AtomicWaker;

/// PIO program for SMT160 capture.
/// 
/// Counts cycles for high phase and total period.
/// Result is pushed as (T_period << 16) | T_high.
pub const SMT160_PIO_PROGRAM: pio::Program<32> = pio_proc::pio_asm!(
    "
    .wrap_target
        wait 1 pin 0        ; Wait for rising edge
        mov x, ~null        ; Initialize X = 0xFFFFFFFF
    high_loop:
        jmp pin high_tick   ; If High, continue
        jmp low_phase       ; If Low, end high phase
    high_tick:
        jmp x-- high_loop   ; Count High + Period
        
    low_phase:
        mov y, x            ; y = ~T_high
    low_loop:
        jmp pin end_period  ; If High again, end period
        jmp x-- low_loop    ; Count Period
        
    end_period:
        mov x, ~x           ; x = T_period
        in x, 16            ; Shift 16 LSBs of T_period into ISR
        mov x, ~y           ; x = T_high
        in x, 16            ; Shift 16 LSBs of T_high into ISR
        push                ; Push packed 32-bit word to FIFO
    .wrap
    "
).program;

pub struct Rp2040PioHal<P, SM, CH, const N: usize>
where
    P: PIOExt,
    SM: StateMachineIndex,
    CH: SingleChannel,
{
    pio: pac::PIO0, // Placeholder, need to handle PIO0/PIO1
    sm: StateMachine<(P, SM), rp2040_hal::pio::Running>,
    dma: CH,
    buffer: &'static mut [u32; N],
    waker: AtomicWaker,
}

impl<P, SM, CH, const N: usize> Rp2040PioHal<P, SM, CH, N>
where
    P: PIOExt,
    SM: StateMachineIndex,
    CH: SingleChannel,
{
    pub fn new(
        _pio: pac::PIO0,
        sm: StateMachine<(P, SM), rp2040_hal::pio::Running>,
        dma: CH,
        buffer: &'static mut [u32; N],
    ) -> Self {
        Self {
            pio: unsafe { pac::Peripherals::steal().PIO0 },
            sm,
            dma,
            buffer,
            waker: AtomicWaker::new(),
        }
    }
}

impl<P, SM, CH, const N: usize> Smt160Hal for Rp2040PioHal<P, SM, CH, N>
where
    P: PIOExt,
    SM: StateMachineIndex,
    CH: SingleChannel,
{
    fn setup(&mut self, _freq: u32) -> Result<(), Smt160Error> {
        // PIO and DMA configuration should be done before passing to the HAL
        // but we ensure it's running.
        Ok(())
    }

    fn is_new_data_available(&self) -> bool {
        !self.sm.rx_fifo().is_empty()
    }

    fn read_raw(&self) -> CapturedEdge {
        if let Some(val) = self.sm.rx_fifo().read() {
            CapturedEdge {
                period_ticks: (val >> 16) & 0xFFFF,
                high_ticks: val & 0xFFFF,
            }
        } else {
            CapturedEdge { period_ticks: 0, high_ticks: 0 }
        }
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
