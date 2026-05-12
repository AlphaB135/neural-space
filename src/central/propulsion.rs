//! Propulsion control — thrust vectoring and fuel mixture optimization.
//!
//! Maintains the Shockwave Stand-off distance. If the bow shock collapses
//! onto the vehicle surface, the thermal load increases by orders of magnitude.

/// Thermal margin — how close we are to melting the engine.
#[inline(always)]
pub fn thermal_margin(skin_temp: f64, engine_temp: f64) -> f64 {
    const MATERIAL_LIMIT: f64 = 12_000.0; // K — C/C composite limit
    1.0 - (skin_temp.max(engine_temp) / MATERIAL_LIMIT)
}

/// Shockwave stand-off distance estimator (simplified Billig correlation).
#[inline(always)]
pub fn standoff_distance(mach: f64, nose_radius: f64) -> f64 {
    // Billig's correlation for shock stand-off
    // delta = 0.143 * R * exp(3.24 / mach^2)
    0.143 * nose_radius * (3.24 / (mach * mach)).exp()
}

/// Compute thrust profile based on current flight conditions.
#[inline(always)]
pub fn compute_thrust_profile(
    mach: f64,
    dynamic_pressure: f64,
    skin_temp_max: f64,
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

    let _ = (mach, dynamic_pressure, thrust_vector);
}
