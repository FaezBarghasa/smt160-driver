//! USB-CDC Serial Telemetry Streamer for SMT160.

#![cfg(feature = "telemetry")]

use crate::{Reading, IndustrialHealth};
use core::fmt::Write;
use usbd_serial::SerialPort;
use usb_device::bus::UsbBus;

/// A high-speed telemetry streamer utilizing the USB-CDC (Serial) interface.
/// 
/// # Summary
/// Formats and transmits sensor readings and health metrics as human-readable 
/// CSV-like frames over a virtual COM port.
pub struct TelemetryStreamer<'a, B: UsbBus> {
    serial_port: SerialPort<'a, B>,
}

impl<'a, B: UsbBus> TelemetryStreamer<'a, B> {
    /// Creates a new telemetry streamer instance.
    pub fn new(serial_port: SerialPort<'a, B>) -> Self {
        Self { serial_port }
    }

    /// Transmits a standardized telemetry frame over the serial port.
    /// 
    /// # Summary
    /// Transmits the temperature, operational status, and health metrics.
    /// 
    /// # Format
    /// `TEMP:<Celsius>,STATUS:<Flags>,JITTER:<Ticks>,FREQ_DRIFT:<Hz>\r\n`
    /// 
    /// # Usage Example
    /// ```
    /// streamer.stream_frame(current_reading, health_metrics);
    /// ```
    pub fn stream_frame(&mut self, reading: Reading, health: IndustrialHealth) {
        let mut string_buffer = [0u8; 128];
        let mut buffer_wrapper = StringBufferWrapper { 
            buffer: &mut string_buffer, 
            offset: 0 
        };
        
        let _ = write!(
            buffer_wrapper, 
            "TEMP:{:.4},STATUS:{:?},JITTER:{},FAULTS:{}\r\n", 
            reading.temperature_celsius.to_num::<f32>(), 
            reading.status,
            health.jitter_ticks,
            health.hardware_fault_count
        );
        
        let valid_length = buffer_wrapper.offset;
        let _ = self.serial_port.write(&string_buffer[..valid_length]);
    }
}

/// Internal helper for safe string formatting into a fixed-size byte buffer.
struct StringBufferWrapper<'a> {
    buffer: &'a mut [u8],
    offset: usize,
}

impl<'a> core::fmt::Write for StringBufferWrapper<'a> {
    fn write_str(&mut self, text_fragment: &str) -> core::fmt::Result {
        let fragment_bytes = text_fragment.as_bytes();
        let remaining_space = self.buffer.len() - self.offset;
        
        if fragment_bytes.len() > remaining_space {
            return Err(core::fmt::Error);
        }
        
        self.buffer[self.offset..self.offset + fragment_bytes.len()].copy_from_slice(fragment_bytes);
        self.offset += fragment_bytes.len();
        Ok(())
    }
}
