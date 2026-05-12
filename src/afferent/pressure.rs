//! Pressure sensor — stagnation and static pressure monitoring.
//!
//! Hypersonic flight creates extreme pressure differentials.
//! Stagnation pressure behind the bow shock is the primary indicator
//! of boundary layer health.

use super::SensorSample;

/// Pressure reading types.
#[repr(u8)]
pub enum PressureType {
    Stagnation = 0,
    StaticUpper = 1,
    StaticLower = 2,
    PitotPort = 3,
    EngineChamber = 4,
}

/// Pressure thresholds in Pascals.
pub const PRESSURE_MAX: f64 = 1_200_000.0; // ~12 atm at Mach 32
pub const ENGINE_CHAMBER_MAX: f64 = 25_000_000.0; // ~250 atm combustion chamber

/// Dynamic pressure (q) from stagnation and static readings.
#[inline(always)]
pub fn dynamic_pressure(stagnation: &SensorSample, static_p: &SensorSample) -> f64 {
    stagnation.value - static_p.value
}

/// Mach number estimate from pressure ratio (Rayleigh pitot formula simplified).
#[inline(always)]
pub fn estimate_mach(stagnation: &SensorSample, static_p: &SensorSample) -> f64 {
    let ratio = stagnation.value / static_p.value;
    // Simplified isentropic relation for hypersonic regime
    ((ratio - 1.0) * 5.0 / 7.0).max(0.0).sqrt() * 2.236 // approximate
}
