#![no_std]
use core::sync::atomic::AtomicU64;
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! { loop {} }
