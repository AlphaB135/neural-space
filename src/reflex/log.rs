//! Fault logging — circular buffer for post-flight analysis.
//!
//! 64-entry static circular buffer. No heap allocation.
//! Deterministic size, bounded write time (~30ns).
//!
//! Overwrites oldest entry when full — preserves recent fault history
//! for incident investigation.

use super::{FaultParam, ReflexAction, Severity};

/// Fault entry — one logged event.
///
/// Captured for post-flight analysis to understand what happened
/// in the critical seconds before incident.
#[repr(C, align(64))]
#[derive(Clone, Copy)]
pub struct FaultEntry {
    /// Which parameter failed
    pub param: FaultParam,
    /// Severity tier at time of fault
    pub severity: Severity,
    /// Action taken
    pub action: ReflexAction,
    /// Cycle counter timestamp (for timing analysis)
    pub timestamp: u64,
    /// Actual sensor value that triggered fault
    pub value: f64,
    /// Threshold we exceeded
    pub threshold: f64,
}

impl FaultEntry {
    /// Zero/empty entry (for buffer initialization).
    #[inline(always)]
    pub const fn zeroed() -> Self {
        Self {
            param: FaultParam::Thermal,
            severity: Severity::Minor,
            action: ReflexAction::NoAction,
            timestamp: 0,
            value: 0.0,
            threshold: 0.0,
        }
    }
}

/// Fault log — 64-entry circular buffer.
///
/// Static allocation, no heap. Deterministic write time.
/// Overwrites oldest when full (overflow flag indicates wrap).
#[repr(C, align(64))]
pub struct FaultLog {
    entries: [FaultEntry; 64],
    index: u8,      // Current write position (0-63)
    count: u8,      // Total entries written (wraps at 255)
    overflow: bool, // True if we wrapped around
}

impl FaultLog {
    /// Create new empty fault log.
    #[inline(always)]
    pub const fn new() -> Self {
        Self {
            entries: [FaultEntry::zeroed(); 64],
            index: 0,
            count: 0,
            overflow: false,
        }
    }

    /// Log a fault — non-blocking, overwrite oldest if full.
    ///
    /// Write time: ~30ns (array index + store)
    ///
    /// # Arguments
    /// - `entry`: Fault entry to log
    #[inline(always)]
    pub fn log(&mut self, entry: FaultEntry) {
        let idx = self.index as usize;
        self.entries[idx] = entry;

        // Advance index (wraps at 64)
        self.index = if self.index < 63 { self.index + 1 } else { 0 };
        self.count = self.count.saturating_add(1);

        // Set overflow flag on wraparound
        if self.index == 0 {
            self.overflow = true;
        }
    }

    /// Read all entries (for post-flight analysis).
    ///
    /// Returns reference to entire buffer.
    /// Note: may contain stale data if not yet filled.
    #[inline(always)]
    pub fn read_all(&self) -> &[FaultEntry] {
        &self.entries
    }

    /// Get number of entries written (may wrap at 255).
    #[inline(always)]
    pub fn entry_count(&self) -> u8 {
        self.count
    }

    /// Check if buffer has overflowed (wrapped around).
    #[inline(always)]
    pub fn has_overflow(&self) -> bool {
        self.overflow
    }

    /// Clear log (reset to empty state).
    #[inline(always)]
    pub fn clear(&mut self) {
        self.index = 0;
        self.count = 0;
        self.overflow = false;
    }
}

/// Read cycle counter — high-resolution timestamp.
///
/// In a real system, this reads the CPU cycle counter (CNTFRQ_EL0 on ARMv8).
/// For now, returns a placeholder.
///
/// TODO: Implement actual cycle counter read for target hardware.
#[inline(always)]
pub fn read_cycle_counter() -> u64 {
    // Placeholder: would use mrs instruction on ARM
    // mrs x0, cntvct_el0
    0
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::{ReflexPriority, ReflexTrigger};

    #[test]
    fn test_log_single_entry() {
        let mut log = FaultLog::new();

        let entry = FaultEntry {
            param: FaultParam::Thermal,
            severity: Severity::Minor,
            action: ReflexAction::NoAction,
            timestamp: 100,
            value: 5000.0,
            threshold: 11000.0,
        };

        log.log(entry);

        assert_eq!(log.entry_count(), 1);
        assert!(!log.has_overflow());
        assert_eq!(log.read_all()[0].timestamp, 100);
    }

    #[test]
    fn test_log_wraparound() {
        let mut log = FaultLog::new();

        // Fill buffer completely
        for i in 0..64 {
            log.log(FaultEntry {
                param: FaultParam::Thermal,
                severity: Severity::Minor,
                action: ReflexAction::NoAction,
                timestamp: i as u64,
                value: 0.0,
                threshold: 0.0,
            });
        }

        assert_eq!(log.entry_count(), 64);
        assert!(log.has_overflow());
        assert_eq!(log.index, 0);  // Wrapped back to start

        // One more entry should overwrite position 0
        log.log(FaultEntry {
            param: FaultParam::Strain,
            severity: Severity::Major,
            action: ReflexAction::ThrottleEmergency,
            timestamp: 999,
            value: 0.0,
            threshold: 0.0,
        });

        assert_eq!(log.read_all()[0].timestamp, 999);  // Overwritten
        assert_eq!(log.read_all()[1].timestamp, 1);   // Next entry intact
    }

    #[test]
    fn test_log_clear() {
        let mut log = FaultLog::new();

        log.log(FaultEntry {
            param: FaultParam::Thermal,
            severity: Severity::Minor,
            action: ReflexAction::NoAction,
            timestamp: 100,
            value: 0.0,
            threshold: 0.0,
        });

        log.clear();

        assert_eq!(log.entry_count(), 0);
        assert!(!log.has_overflow());
        assert_eq!(log.index, 0);
    }
}
