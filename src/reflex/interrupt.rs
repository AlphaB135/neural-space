//! Hardware interrupt handlers for the reflex loop.
//!
//! These are naked functions that run in interrupt context.
//! They must be minimal — save state, fire reflex, return.

/// Reflex interrupt vector table entry.
#[repr(C)]
pub struct InterruptVector {
    pub handler: unsafe extern "C" fn(),
    pub priority: u8,
}

/// Hardware interrupt source IDs.
pub const IRQ_THERMAL_CRITICAL: u16 = 0x10;
pub const IRQ_STRAIN_CRITICAL: u16 = 0x11;
pub const IRQ_PRESSURE_CRITICAL: u16 = 0x12;
pub const IRQ_GYRO_SATURATION: u16 = 0x13;
pub const IRQ_WATCHDOG: u16 = 0x1F;

/// Reflex response time budget — everything must complete within this.
pub const REFLEX_DEADLINE_NS: u64 = 500; // 500 nanoseconds

/// Acknowledge interrupt — write to interrupt controller.
///
/// # Safety
/// This function writes to hardware MMIO registers.
/// Callers must ensure the register address is valid.
#[inline(always)]
pub unsafe fn acknowledge(irq: u16) {
    let _ = irq;
    // TODO: Write to GIC (Generic Interrupt Controller) distributor
    // register to clear the pending interrupt.
    //
    // Example for ARMv8 GIC:
    // let gicd_icpendr = 0x08000000 as *mut u32; // GICD_ICPENDR0
    // gicd_icpendr.write_volatile(1 << (irq % 32));
}
