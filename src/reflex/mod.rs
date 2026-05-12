//! Spinal Reflex Loop — The "Oh Shit" Protocol
//!
//! Hardware-level interrupts. If structural failure is imminent or a hard
//! start occurs in the turbopump, this loop bypasses the brain entirely.
//! It cuts the fuel and triggers the ejection sequence in microseconds.
//! It does not ask for permission.
//!
//! References: [Ref 7] (fault-tolerant tracking, prescribed error bounds),
//! [Ref 14] (DTIC stability and control of hypersonic vehicles)

pub mod interrupt;
pub mod ejection;

/// Reflex priority — lower number = higher priority.
#[repr(u8)]
pub enum ReflexPriority {
    StructuralFailure = 0, // Immediate ejection
    HardStart = 1,         // Engine destruction imminent
    ThermalBreach = 2,     // Skin temp exceeding material limits
    GyroSaturation = 3,    // Loss of attitude reference
    HydraulicLoss = 4,     // Control authority degrading
}

/// Reflex action — the autonomic response, no thinking required.
#[repr(u8)]
pub enum ReflexAction {
    Eject = 0,
    EngineShutdown = 1,
    ThrottleEmergency = 2,
    SurfaceNeutral = 3,
    NoAction = 255,
}

/// Reflex trigger — what fired the reflex.
#[repr(C)]
pub struct ReflexTrigger {
    pub priority: ReflexPriority,
    pub action: ReflexAction,
    pub source_channel: u8,
    pub value: f64,
    pub threshold: f64,
}

/// The reflex loop — checks all critical conditions and fires if needed.
/// This runs at the highest interrupt priority, before the main control loop.
#[inline(always)]
pub fn check_reflexes(
    thermal_max: f64,
    strain_max: f64,
    chamber_pressure: f64,
    gyro_rate: f64,
) -> Option<ReflexTrigger> {
    // Structural failure — eject immediately
    if strain_max >= 14_000.0 {
        return Some(ReflexTrigger {
            priority: ReflexPriority::StructuralFailure,
            action: ReflexAction::Eject,
            source_channel: 0,
            value: strain_max,
            threshold: 14_000.0,
        });
    }

    // Thermal breach — shutdown engine
    if thermal_max >= 11_000.0 {
        return Some(ReflexTrigger {
            priority: ReflexPriority::ThermalBreach,
            action: ReflexAction::EngineShutdown,
            source_channel: 1,
            value: thermal_max,
            threshold: 11_000.0,
        });
    }

    // Hard start — chamber pressure spike
    if chamber_pressure >= 30_000_000.0 {
        return Some(ReflexTrigger {
            priority: ReflexPriority::HardStart,
            action: ReflexAction::EngineShutdown,
            source_channel: 2,
            value: chamber_pressure,
            threshold: 30_000_000.0,
        });
    }

    // Gyro saturation — loss of attitude reference
    if gyro_rate.abs() >= 50.0 {
        return Some(ReflexTrigger {
            priority: ReflexPriority::GyroSaturation,
            action: ReflexAction::SurfaceNeutral,
            source_channel: 3,
            value: gyro_rate,
            threshold: 50.0,
        });
    }

    None
}
