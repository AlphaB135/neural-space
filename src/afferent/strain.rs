//! Structural strain gauge monitoring.
//!
//! Measures airframe flex, G-loading, and structural deformation.
//! At extreme G-forces, the airframe bends in ways that FEA models
//! only approximate. The strain gauges tell the truth.

use super::SensorSample;

/// Strain gauge locations along the airframe.
#[repr(u8)]
pub enum StrainLocation {
    ForwardBulkhead = 0,
    WingSparPort = 1,
    WingSparStarboard = 2,
    EngineMountForward = 3,
    EngineMountAft = 4,
    TailBoom = 5,
    FuselageMidSection = 6,
}

/// Microstrain limits — beyond this, the material yields.
pub const YIELD_STRAIN: f64 = 8_000.0; // microstrain for titanium alloy
pub const ULTIMATE_STRAIN: f64 = 14_000.0; // catastrophic failure

/// G-load from strain differential (simplified).
#[inline(always)]
pub fn g_load_from_strain(sample: &SensorSample) -> f64 {
    // Microstrain to G conversion — calibrated per-location
    (sample.value / YIELD_STRAIN) * 9.0
}

/// Check if structural yield is imminent.
#[inline(always)]
pub fn is_yield_imminent(sample: &SensorSample) -> bool {
    sample.value >= YIELD_STRAIN * 0.85 // 85% yield = danger zone
}

/// Check if catastrophic failure is imminent.
#[inline(always)]
pub fn is_failure_imminent(sample: &SensorSample) -> bool {
    sample.value >= ULTIMATE_STRAIN * 0.90 // 90% ultimate = eject now
}
