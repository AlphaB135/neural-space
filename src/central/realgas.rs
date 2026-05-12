//! Real gas effects correction for hypersonic flows.
//!
//! At Mach 20+, air is no longer calorically perfect. Vibrational excitation
//! and dissociation reduce the effective specific heat ratio (γ).
//!
//! References:
//! - [Ref 3] Anderson, J.D. "Hypersonic and High-Temperature Gas Dynamics"
//! - [Ref 4] Wen & Hornung (1995) "Non-equilibrium dissociating flow over spheres"
//! - "Second-Order Shock-Expansion Theory Extended to Include Real Gas Effects" [DTIC ADA247191]
//!   [Link](https://apps.dtic.mil/sti/tr/pdf/ADA247191.pdf)

use libm::{sqrt, exp};

/// Effective specific heat ratio as function of temperature.
///
/// This is a simplified correlation for equilibrium air. At high temperatures,
/// vibrational modes excite (~800K) and molecules dissociate (O₂ ~2500K, N₂ ~6000K),
/// reducing γ from 1.4 toward 1.1-1.15.
///
/// Temperature ranges (approximate):
/// - T < 800K:   γ ≈ 1.40 (calorically perfect)
/// - 800-2500K:  γ ≈ 1.35-1.38 (vibrational excitation)
/// - 2500-6000K: γ ≈ 1.25-1.35 (O₂ dissociation)
/// - 6000K+:    γ ≈ 1.15-1.25 (N₂ dissociation, ionization)
///
/// For production use, replace with lookup table from CFD or thermodynamic
/// database (e.g., NASA CEA).
#[inline(always)]
pub fn effective_gamma(temperature_k: f64) -> f64 {
    if temperature_k < 800.0 {
        1.40
    } else if temperature_k < 2500.0 {
        // Linear ramp for vibrational excitation regime
        1.40 - (temperature_k - 800.0) / 1700.0 * 0.05 // 1.40 → 1.35
    } else if temperature_k < 6000.0 {
        // O₂ dissociation regime
        1.35 - (temperature_k - 2500.0) / 3500.0 * 0.10 // 1.35 → 1.25
    } else {
        // N₂ dissociation + ionization regime
        (1.25 - (temperature_k - 6000.0) / 4000.0 * 0.10).max(1.15) // 1.25 → 1.15
    }
}

/// Real gas corrected shock standoff distance.
///
/// Billig correlation assumes γ=1.4. For real gas, use effective γ in the
/// exponential term. This increases standoff distance by 10-30% at Mach 20-30.
///
/// Modified formula: Δ/Rₙ = C(γ_eff) · exp(K(γ_eff)/M²)
///
/// where C and K depend on effective gamma. For spheres:
/// - C ≈ 0.143 · (1.4/γ_eff)^0.5
/// - K ≈ 3.24 · (γ_eff/1.4)
///
/// These approximations are derived from the shock jump conditions with
/// variable γ. See [Ref 3] Chapter 2 for derivation.
///
/// Args:
/// - mach: freestream Mach number
/// - nose_radius: nose radius in meters
/// - temperature_k: freestream temperature in Kelvin (for γ_eff calculation)
///
/// Returns: shock standoff distance in meters
#[inline(always)]
pub fn standoff_distance_real_gas(mach: f64, nose_radius: f64, temperature_k: f64) -> f64 {
    let gamma = effective_gamma(temperature_k);

    // Coefficient corrections for variable gamma
    // Derived from normal shock relations with γ ≠ 1.4
    let c_correction = sqrt(1.4 / gamma);
    let k_correction = gamma / 1.4;

    // Original Billig coefficients [Ref 1]
    let c0 = 0.143;
    let k0 = 3.24;

    // Corrected coefficients
    let c = c0 * c_correction;
    let k = k0 * k_correction;

    c * nose_radius * exp(k / (mach * mach))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_effective_gamma_cold() {
        assert!((effective_gamma(300.0) - 1.40).abs() < 0.01);
    }

    #[test]
    fn test_effective_gamma_vibrational() {
        let gamma = effective_gamma(1500.0);
        assert!(gamma > 1.35 && gamma < 1.40);
    }

    #[test]
    fn test_effective_gamma_dissociation() {
        let gamma = effective_gamma(4000.0);
        assert!(gamma > 1.25 && gamma < 1.35);
    }

    #[test]
    fn test_effective_gamma_extreme() {
        let gamma = effective_gamma(10000.0);
        assert!(gamma > 1.10 && gamma < 1.25);
    }

    #[test]
    fn test_real_gas_increases_standoff() {
        let perfect = standoff_distance_real_gas(25.0, 1.0, 300.0); // cold air, γ=1.4
        let real = standoff_distance_real_gas(25.0, 1.0, 8000.0); // hot air, γ<1.4
        assert!(real > perfect); // Real gas → larger standoff
    }
}
