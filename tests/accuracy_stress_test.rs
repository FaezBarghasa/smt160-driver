#![cfg(not(target_arch = "arm"))]

use smt160_driver::hal::{Smt160Hal, CapturedEdge};
use smt160_driver::error::Smt160Error;
use smt160_driver::{Smt160Driver, Config, Smt160Status};
use fixed::types::I32F32;
use core::cell::RefCell;

struct MockHal {
    next_data: RefCell<Option<CapturedEdge>>,
}

impl Smt160Hal for MockHal {
    fn setup(&mut self, _freq: u32) -> Result<(), Smt160Error> {
        Ok(())
    }

    fn is_new_data_available(&self) -> bool {
        self.next_data.borrow().is_some()
    }

    fn read_raw(&self) -> CapturedEdge {
        self.next_data.borrow_mut().take().unwrap()
    }
}

#[test]
fn test_jitter_detection() {
    let hal = MockHal { next_data: RefCell::new(None) };
    let mut driver = Smt160Driver::new(hal, Config::industrial())
        .init(1000)
        .unwrap();

    // First sample: stable
    *driver.hal_mut().next_data.borrow_mut() = Some(CapturedEdge { period_ticks: 1000, high_ticks: 437 });
    driver.read_temperature();
    assert!(!driver.status().contains(Smt160Status::JITTER_DETECTED));

    // Second sample: 0.6% jitter (> 0.5% industrial threshold)
    *driver.hal_mut().next_data.borrow_mut() = Some(CapturedEdge { period_ticks: 1006, high_ticks: 440 });
    driver.read_temperature();
    assert!(driver.status().contains(Smt160Status::JITTER_DETECTED));
}

#[test]
fn test_adaptive_filtering() {
    let hal = MockHal { next_data: RefCell::new(None) };
    let mut driver = Smt160Driver::new(hal, Config::industrial())
        .init(1000)
        .unwrap();

    // Sample 1: 25°C
    *driver.hal_mut().next_data.borrow_mut() = Some(CapturedEdge { period_ticks: 1000, high_ticks: 437 });
    let _temp1 = driver.read_temperature().unwrap();
    
    // Sample 2: Sudden jump to 35°C (> 5°C deviation, should use alpha=0.8)
    // DC for 35°C: T = (DC - 0.320) / 0.0047 -> 35 * 0.0047 + 0.320 = 0.4845
    *driver.hal_mut().next_data.borrow_mut() = Some(CapturedEdge { period_ticks: 1000, high_ticks: 484 });
    let temp2 = driver.read_temperature().unwrap();
    
    // Expected: 25 + 0.8 * (35 - 25) = 33
    assert!(temp2 > I32F32::from_num(32) && temp2 < I32F32::from_num(34));
}

#[test]
fn test_invalid_signal() {
    let hal = MockHal { next_data: RefCell::new(None) };
    let mut driver = Smt160Driver::new(hal, Config::industrial())
        .init(1000)
        .unwrap();

    // Active > Period
    *driver.hal_mut().next_data.borrow_mut() = Some(CapturedEdge { period_ticks: 1000, high_ticks: 1200 });
    let result = driver.read_temperature();
    
    assert!(result.is_none());
    assert!(driver.status().contains(Smt160Status::OUT_OF_BOUNDS));
}

#[test]
fn test_reinit_resets_state() {
    let hal = MockHal { next_data: RefCell::new(None) };
    let mut driver = Smt160Driver::new(hal, Config::industrial())
        .init(1000)
        .unwrap();

    // Set some state
    *driver.hal_mut().next_data.borrow_mut() = Some(CapturedEdge { period_ticks: 1000, high_ticks: 437 });
    driver.read_temperature();
    
    // Simulate some ticks and status
    for _ in 0..1000 {
        driver.read_temperature();
    }
    assert!(driver.status().contains(Smt160Status::SENSOR_TIMEOUT));
    
    // Re-init
    driver.reinit(1000).unwrap();
    
    assert_eq!(driver.status().bits(), 0);
}
