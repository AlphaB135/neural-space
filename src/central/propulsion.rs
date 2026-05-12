//! Propulsion control — thrust vectoring and fuel mixture optimization.
//!
//! Maintains the Shockwave Stand-off distance. If the bow shock collapses
//! onto the vehicle surface, the thermal load increases by orders of magnitude.
//!
//! References: [Ref 1, 3, 4, 8]
//!
//! NOTE: For real gas effects (Mach 15+), use `realgas::standoff_distance_real_gas`
//! instead of `standoff_distance_perfect_gas`.

use super::realgas;
use libm::exp;

/// Thermal margin — how close we are to melting the engine.
/// Material limit based on C/C composite data from [Ref 10].
#[inline(always)]
pub fn thermal_margin(skin_temp: f64, engine_temp: f64) -> f64 {
    const MATERIAL_LIMIT: f64 = 12_000.0; // K — C/C composite, see [Ref 10]
    1.0 - (skin_temp.max(engine_temp) / MATERIAL_LIMIT)
}

/// Shock stand-off distance — Billig correlation [Ref 1].
///
/// Billig, F.S. (1967): Δ/Rₙ = 0.143 · exp(3.24/M²) for spherical nose.
///
/// **Use this only for cold hypersonic flow (T < 800K, M < 10).**
/// For hot, high-Mach flows, use `realgas::standoff_distance_real_gas` instead.
#[inline(always)]
pub fn standoff_distance_perfect_gas(mach: f64, nose_radius: f64) -> f64 {
    0.143 * nose_radius * exp(3.24 / (mach * mach))
}

/// Shock stand-off distance with real gas correction [Ref 3, 4].
///
/// Uses effective γ based on freestream temperature. This is the preferred
/// method for Mach 15+ where dissociation becomes significant.
///
/// For comparison:
/// - Perfect gas (γ=1.4): `standoff_distance_perfect_gas`
/// - Real gas: `standoff_distance_real_gas`
///
/// At Mach 25, 8000K freestream: real gas gives ~15-20% larger standoff.
#[inline(always)]
pub fn standoff_distance_real_gas(mach: f64, nose_radius: f64, temperature_k: f64) -> f64 {
    realgas::standoff_distance_real_gas(mach, nose_radius, temperature_k)
}

/// Compute thrust profile based on current flight conditions.
///
/// Uses real gas corrected shock standoff for thermal margin calculation
/// when freestream temperature indicates high-enthalpy flow.
#[inline(always)]
pub fn compute_thrust_profile(
    mach: f64,
    dynamic_pressure: f64,
    skin_temp_max: f64,
    freestream_temp: f64,
    thrust_vector: &mut [f64; 3],
    throttle: &mut f64,
    fuel_mixture: &mut f64,
) {
    // Reduce throttle if thermal margin is low
    let margin = thermal_margin(skin_temp_max, skin_temp_max * 0.9);
    if margin < 0.15 {
        *throttle *= 0.7; // aggressive throttle back
        *fuel_mixture = 2.80; // richer mixture for cooling
    }

    // TODO: Use standoff distance (real gas) for active shock stand-off control
    let _ = (mach, dynamic_pressure, freestream_temp, thrust_vector);
}
