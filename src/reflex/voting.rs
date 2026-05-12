//! Triple Modular Redundancy (TMR) voting for safety-critical fault detection.
//!
//! At Mach 32, sensor disagreement = uncertainty = death.
//! This module implements 2-out-of-3 voting with safety escalation:
//! - If 2/3 sensors agree → use consensus value
//! - If no majority → ESCALATE to critical (safety first)
//!
//! References: [Ref 7] (fault-tolerant tracking), [Ref 14] (DTIC stability)

/// Triple sensor readings for one parameter.
/// A = primary, B = redundant 1, C = redundant 2
#[repr(C, align(32))]
#[derive(Clone, Copy)]
pub struct SensorTriplet<T> {
    pub a: T,
    pub b: T,
    pub c: T,
}

impl<T> SensorTriplet<T> {
    /// Create new sensor triplet.
    #[inline(always)]
    pub const fn new(a: T, b: T, c: T) -> Self {
        Self { a, b, c }
    }
}

/// Voting result with safety escalation.
#[repr(C, align(16))]
#[derive(Clone, Copy)]
pub enum VoteResult<T> {
    /// 2/3 sensors agreed — value is valid
    Consensus(T),
    /// No majority — unsafe, escalate response
    Escalate,
}

/// Vote on 3 sensor readings with safety fallback.
///
/// # Arguments
/// - `triplet`: Three sensor readings
/// - `tolerance`: Maximum allowed difference for agreement (absolute)
///
/// # Returns
/// - `Consensus(value)` if 2/3 sensors agree within tolerance
/// - `Escalate` if no majority (all three disagree)
///
/// # Safety
/// This is the core safety-critical function. If voting fails,
/// we MUST escalate — uncertainty at Mach 32 is fatal.
///
/// # Algorithm
/// 1. Check pairwise agreement: (A≈B), (B≈C), (A≈C)
/// 2. If A≈B → use (A+B)/2
/// 3. Else if B≈C → use (B+C)/2
/// 4. Else if A≈C → use (A+C)/2
/// 5. Else → ESCALATE (no majority)
///
/// # Performance
/// ~30ns per call: 3 × (sub + abs + cmp) + avg
///
/// # Example
/// ```
/// let sensors = SensorTriplet::new(1000.0, 1005.0, 2000.0);
/// let result = vote_triplet(&sensors, 50.0);
/// assert!(matches!(result, VoteResult::Consensus(v) if (v - 1002.5).abs() < 1.0));
/// ```
#[inline(always)]
pub fn vote_triplet(
    triplet: &SensorTriplet<f64>,
    tolerance: f64,
) -> VoteResult<f64> {
    let a = triplet.a;
    let b = triplet.b;
    let c = triplet.c;

    // Sanity check: reject NaN/Inf immediately
    if !a.is_finite() || !b.is_finite() || !c.is_finite() {
        return VoteResult::Escalate;
    }

    // Pairwise comparison (A≈B, B≈C, A≈C)
    // Using libm::fabs would require extern crate import
    // Use intrinsic f64::abs() which is available in core
    let ab_agree = (a - b).abs() <= tolerance;
    let bc_agree = (b - c).abs() <= tolerance;
    let ac_agree = (a - c).abs() <= tolerance;

    // 2-out-of-3 voting logic
    if ab_agree {
        // A and B agree → use their average
        VoteResult::Consensus((a + b) * 0.5)
    } else if bc_agree {
        // B and C agree → use their average
        VoteResult::Consensus((b + c) * 0.5)
    } else if ac_agree {
        // A and C agree → use their average
        VoteResult::Consensus((a + c) * 0.5)
    } else {
        // No majority → ESCALATE TO SAFETY
        // At Mach 32, uncertainty = death
        VoteResult::Escalate
    }
}

/// Tolerance values for each parameter type.
/// Based on sensor specifications + noise floor + thermal gradients.
pub mod tolerances {
    /// Thermal sensor tolerance: ±50 K
    /// Accounts for: sensor noise ±20K, thermal gradient ±30K
    pub const THERMAL: f64 = 50.0;

    /// Strain sensor tolerance: ±500 Pa
    /// Accounts for: load cell resolution ±200Pa, vibration ±300Pa
    pub const STRAIN: f64 = 500.0;

    /// Pressure sensor tolerance: ±100 kPa
    /// Accounts for: transducer accuracy ±50kPa, line noise ±50kPa
    pub const PRESSURE: f64 = 100_000.0;

    /// Gyro sensor tolerance: ±0.5 rad/s
    /// Accounts for: IMU noise floor ±0.3rad/s, drift ±0.2rad/s
    pub const GYRO: f64 = 0.5;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vote_all_agree() {
        let triplet = SensorTriplet::new(1000.0, 1005.0, 1002.0);
        let result = vote_triplet(&triplet, 50.0);
        assert!(matches!(result, VoteResult::Consensus(v) if (v - 1002.5).abs() < 1.0));
    }

    #[test]
    fn test_vote_ab_agree() {
        // A and B agree, C disagrees
        let triplet = SensorTriplet::new(1000.0, 1010.0, 2000.0);
        let result = vote_triplet(&triplet, 50.0);
        assert!(matches!(result, VoteResult::Consensus(v) if (v - 1005.0).abs() < 1.0));
    }

    #[test]
    fn test_vote_bc_agree() {
        // B and C agree, A disagrees
        let triplet = SensorTriplet::new(2000.0, 1000.0, 1010.0);
        let result = vote_triplet(&triplet, 50.0);
        assert!(matches!(result, VoteResult::Consensus(v) if (v - 1005.0).abs() < 1.0));
    }

    #[test]
    fn test_vote_ac_agree() {
        // A and C agree, B disagrees
        let triplet = SensorTriplet::new(1000.0, 2000.0, 1010.0);
        let result = vote_triplet(&triplet, 50.0);
        assert!(matches!(result, VoteResult::Consensus(v) if (v - 1005.0).abs() < 1.0));
    }

    #[test]
    fn test_vote_no_majority() {
        // All three disagree
        let triplet = SensorTriplet::new(1000.0, 2000.0, 3000.0);
        let result = vote_triplet(&triplet, 100.0);
        assert!(matches!(result, VoteResult::Escalate));
    }

    #[test]
    fn test_vote_nan_input() {
        let triplet = SensorTriplet::new(f64::NAN, 1000.0, 1005.0);
        let result = vote_triplet(&triplet, 50.0);
        assert!(matches!(result, VoteResult::Escalate));
    }

    #[test]
    fn test_vote_infinite_input() {
        let triplet = SensorTriplet::new(f64::INFINITY, 1000.0, 1005.0);
        let result = vote_triplet(&triplet, 50.0);
        assert!(matches!(result, VoteResult::Escalate));
    }
}
