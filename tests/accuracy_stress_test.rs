use smt160_driver::decoder::Smt160Decoder;
use fixed::types::I16F16;

#[test]
fn accuracy_stress_test_jitter() {
    let timer_clock_mhz = 8;
    let mut decoder = Smt160Decoder::new_standalone(timer_clock_mhz);
    
    // SMT160 output at 25C: Duty cycle around 0.32 + 0.0047 * 25 = 0.4375
    // Let's assume a period of 4000 ticks (for 8MHz, that's 2kHz signal, typical for SMT160)
    let period_ticks = 4000u64;
    let active_ticks = 1750u64; // 1750 / 4000 = 0.4375
    
    // Feed 500 noisy pulses
    let mut min_temp = I16F16::MAX;
    let mut max_temp = I16F16::MIN;
    
    // Pseudo-random noise of +/- 2 ticks
    let mut prng_state = 12345u32;
    let mut next_rand = || -> i64 {
        prng_state = prng_state.wrapping_mul(1103515245).wrapping_add(12345);
        ((prng_state >> 16) % 5) as i64 - 2 // -2, -1, 0, 1, 2
    };

    let duty_cycle_offset = smt160_driver::config::DUTY_CYCLE_OFFSET;
    let inverse_step_constant = smt160_driver::config::INVERSE_STEP_CONSTANT;

    // Warm-up to let EMA settle (100 samples)
    for _ in 0..100 {
        let noise_period = next_rand();
        let noise_active = next_rand();
        let p = (period_ticks as i64 + noise_period) as u64;
        let a = (active_ticks as i64 + noise_active) as u64;
        let _ = decoder.process_raw_ticks(p, a, duty_cycle_offset, inverse_step_constant);
    }

    // Now test 500 pulses
    for _ in 0..500 {
        let noise_period = next_rand();
        let noise_active = next_rand();
        let p = (period_ticks as i64 + noise_period) as u64;
        let a = (active_ticks as i64 + noise_active) as u64;
        
        let reading = decoder.process_raw_ticks(p, a, duty_cycle_offset, inverse_step_constant).unwrap();
        
        if reading.temperature_celsius < min_temp {
            min_temp = reading.temperature_celsius;
        }
        if reading.temperature_celsius > max_temp {
            max_temp = reading.temperature_celsius;
        }
    }
    
    let jitter = max_temp - min_temp;
    let max_jitter = I16F16::from_num(0.02);
    
    println!("Min Temp: {}", min_temp);
    println!("Max Temp: {}", max_temp);
    println!("Jitter: {}", jitter);
    
    assert!(jitter < max_jitter, "Filtered jitter {} exceeds maximum allowed {}", jitter, max_jitter);
}
