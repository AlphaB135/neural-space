//! Thermal sensor — skin temperature monitoring.
//!
//! Reads surface thermocouple arrays across the vehicle skin.
//! At Mach 32, skin temp can exceed 10,000 degrees in milliseconds.

use super::SensorSample;

/// Thermal zones — vehicle is divided into discrete monitoring zones.
#[repr(u8)]
pub enum ThermalZone {
    NoseCone = 0,
    ForebodyUpper = 1,
    ForebodyLower = 2,
    WingRootPort = 3,
    WingRootStarboard = 4,
    EngineNacelle = 5,
    AftBody = 6,
}

/// Thermal limits per zone in Kelvin.
pub const THERMAL_LIMITS: [f64; 7] = [
    11_000.0, // NoseCone — takes the brunt
    8_500.0,  // ForebodyUpper
    8_500.0,  // ForebodyLower
    7_200.0,  // WingRootPort
    7_200.0,  // WingRootStarboard
    9_800.0,  // EngineNacelle
    6_000.0,  // AftBody
];

/// Check if a thermal zone exceeds its limit.
#[inline(always)]
pub fn is_exceeded(sample: &SensorSample, zone: ThermalZone) -> bool {
    sample.value >= THERMAL_LIMITS[zone as usize]
}

/// Thermal gradient between two zones — rapid change indicates boundary layer collapse.
#[inline(always)]
pub fn thermal_gradient(a: &SensorSample, b: &SensorSample) -> f64 {
    let dt = (a.timestamp as f64) - (b.timestamp as f64);
    if dt.abs() < 1.0 {
        return 0.0;
    }
    (a.value - b.value) / dt.abs()
}
