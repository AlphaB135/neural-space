//! Afferent Subsystem — Sensory Input
//!
//! Direct Memory Access (DMA) polling of thermal, pressure, and structural
//! strain gauges. No buffering. No middleware. We read the metal directly.

pub mod thermal;
pub mod pressure;
pub mod strain;

/// Raw sensor sample — single atomic reading from the metal.
#[repr(C, align(16))]
pub struct SensorSample {
    pub timestamp: u64,
    pub channel: u8,
    pub value: f64,
    pub checksum: u16,
}

/// Afferent bus — polls all sensor channels in a single deterministic pass.
pub struct AfferentBus {
    channels: [SensorSample; 64],
}

impl AfferentBus {
    pub const fn new() -> Self {
        Self {
            channels: [SensorSample {
                timestamp: 0,
                channel: 0,
                value: 0.0,
                checksum: 0,
            }; 64],
        }
    }

    /// DMA-driven poll — reads all channels in one pass.
    /// Safety: caller must ensure DMA regions are mapped and valid.
    pub unsafe fn poll_dma(&mut self) {
        // DMA polling of sensor registers
        // Each channel reads directly from memory-mapped sensor addresses
        for (i, sample) in self.channels.iter_mut().enumerate() {
            sample.channel = i as u8;
            // Placeholder: real implementation reads from MMIO
            sample.value = 0.0;
            sample.timestamp = Self::cycle_count();
        }
    }

    #[inline(always)]
    fn cycle_count() -> u64 {
        let cycles: u64;
        unsafe { core::arch::asm!("mrs {}, cntvct_el0", out(reg) cycles) };
        cycles
    }
}
