//! Efferent Subsystem — Actuation / Muscles
//!
//! Sub-millisecond hardware manipulation. Fires the liquid rocket actuators
//! and control surfaces exactly when needed.

pub mod actuators;
pub mod surfaces;

/// Efferent command — one atomic actuation instruction.
#[repr(C, align(8))]
pub struct EfferentCommand {
    pub target: u16,
    pub value: f64,
    pub deadline_ns: u64,
}

/// Efferent bus — dispatches commands to hardware in priority order.
pub struct EfferentBus {
    commands: [EfferentCommand; 32],
    count: usize,
}

impl EfferentBus {
    pub const fn new() -> Self {
        Self {
            commands: [EfferentCommand {
                target: 0,
                value: 0.0,
                deadline_ns: 0,
            }; 32],
            count: 0,
        }
    }

    /// Dispatch all pending commands — writes directly to actuator registers.
    /// Safety: caller must ensure actuator MMIO is mapped.
    pub unsafe fn dispatch(&mut self) {
        for i in 0..self.count {
            let _cmd = &self.commands[i];
            // Write to hardware register
        }
        self.count = 0;
    }

    #[inline(always)]
    pub fn enqueue(&mut self, target: u16, value: f64, deadline_ns: u64) {
        if self.count < self.commands.len() {
            self.commands[self.count] = EfferentCommand {
                target,
                value,
                deadline_ns,
            };
            self.count += 1;
        }
    }
}
