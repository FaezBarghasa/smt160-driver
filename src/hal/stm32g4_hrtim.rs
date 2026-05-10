use crate::error::Smt160Error;
use crate::hal::{Smt160Hal, CapturedEdge};
use stm32g4xx_hal::pac;
use embassy_sync::waitqueue::AtomicWaker;

/// STM32G4-specific implementation using High-Resolution Timer (HRTIM).
/// 
/// This HAL leverages the DLL-backed 184ps resolution of the HRTIM for
/// extreme precision capture.
pub struct Stm32G4HrtimHal {
    hrtim: pac::HRTIM_COMMON,
    tim_a: pac::HRTIM_TIMA,
    waker: AtomicWaker,
}

impl Stm32G4HrtimHal {
    pub fn new(hrtim: pac::HRTIM_COMMON, tim_a: pac::HRTIM_TIMA) -> Self {
        Self {
            hrtim,
            tim_a,
            waker: AtomicWaker::new(),
        }
    }
}

impl Smt160Hal for Stm32G4HrtimHal {
    fn setup(&mut self, _freq: u32) -> Result<(), Smt160Error> {
        // 1. Enable DLL for maximum resolution
        self.hrtim.dllcr.modify(|_, w| w.cal().set_bit().calen().set_bit());
        while !self.hrtim.isr.read().dllrdy().bit_is_set() {}

        // 2. Configure Timer A for PWM Capture
        // Prescaler = 1 (Maximum resolution)
        self.tim_a.timacr.modify(|_, w| w.ck_psc().bits(0).cont().set_bit());
        
        // Reset Timer on Capture 1 (Rising edge of signal)
        // Assume External Event 1 (EEV1) is configured as the input
        self.tim_a.rstustr.modify(|_, w| w.cpt1().set_bit());
        
        // Capture 1 on EEV1 (Rising)
        self.tim_a.cpt1ar.modify(|_, w| w.timaeev1().set_bit());
        // Capture 2 on EEV2 (Falling)
        self.tim_a.cpt2ar.modify(|_, w| w.timaeev2().set_bit());

        // 3. Enable Timer A and Capture Interrupts
        self.tim_a.timadier.modify(|_, w| w.cpt1ie().set_bit());
        self.hrtim.mcr.modify(|_, w| w.tamen().set_bit());

        Ok(())
    }

    fn is_new_data_available(&self) -> bool {
        self.tim_a.timaisr.read().cpt1().bit_is_set()
    }

    fn read_raw(&self) -> CapturedEdge {
        // Captured values are 16-bit, but HRTIM resolution makes them effective > 32-bit if prescaled.
        // Here we use the raw capture registers.
        let period = self.tim_a.cpt1ar.read().cpt1x().bits() as u32;
        let high = self.tim_a.cpt2ar.read().cpt2x().bits() as u32;

        // Clear interrupt flags
        self.tim_a.timaicr.write(|w| w.cpt1c().set_bit().cpt2c().set_bit());

        CapturedEdge {
            period_ticks: period as u64,
            high_ticks: high as u64,
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
