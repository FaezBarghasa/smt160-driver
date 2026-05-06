//! USB-CDC Telemetry Stream.

#![cfg(feature = "telemetry")]

use crate::Reading;
use core::fmt::Write;
use usbd_serial::SerialPort;
use usb_device::bus::UsbBus;

/// Telemetry streamer for USB-CDC.
pub struct TelemetryStreamer<'a, B: UsbBus> {
    serial: SerialPort<'a, B>,
}

impl<'a, B: UsbBus> TelemetryStreamer<'a, B> {
    /// Creates a new telemetry streamer.
    pub fn new(serial: SerialPort<'a, B>) -> Self {
        Self { serial }
    }

    /// Polls the USB device. Must be called regularly.
    pub fn poll(&mut self) {
        // Simple poll logic, usually handled by usb_device::UsbDevice
    }

    /// Streams a reading over USB.
    /// Format: "TEMP:XX.XXXX,STATUS:OK\r\n"
    pub fn stream_reading(&mut self, reading: Reading) {
        let mut buf = [0u8; 64];
        let mut wrapper = Wrapper { buf: &mut buf, offset: 0 };
        
        let _ = write!(
            wrapper, 
            "TEMP:{:.4},STATUS:{:?}\r\n", 
            reading.value.to_num::<f32>(), 
            reading.status
        );
        
        let len = wrapper.offset;
        let _ = self.serial.write(&buf[..len]);
    }
}

struct Wrapper<'a> {
    buf: &'a mut [u8],
    offset: usize,
}

impl<'a> core::fmt::Write for Wrapper<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = self.buf.len() - self.offset;
        if bytes.len() > remaining {
            return Err(core::fmt::Error);
        }
        self.buf[self.offset..self.offset + bytes.len()].copy_from_slice(bytes);
        self.offset += bytes.len();
        Ok(())
    }
}
