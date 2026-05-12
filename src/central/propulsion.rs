//! Propulsion control — thrust vectoring and fuel mixture optimization.
//!
//! Maintains the Shockwave Stand-off distance. If the bow shock collapses
//! onto the vehicle surface, the thermal load increases by orders of magnitude.
//!
//! References: [Ref 1, 3, 8]

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
/// Caveat: assumes calorically perfect gas (γ=1.4). At Mach 20+,
/// real gas effects (dissociation, ionization) increase standoff beyond
/// this prediction — see [Ref 3, 4] for correction factors.
#[inline(always)]
pub fn standoff_distance(mach: f64, nose_radius: f64) -> f64 {
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
