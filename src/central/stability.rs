//! Stability control — attitude and angular rate management.
//!
//! Maintains vehicle orientation within tight tolerances despite
//! aerodynamic disturbances at hypersonic speeds.
//!
//! Quaternion-based PID control with fixed gains. Stateless design
//! for deterministic execution bounded to 100ns control cycle.
//!
//! References: [Ref 5, 8, 13] — deterministic learning control,
//! control-oriented modeling, flight dynamics survey.

use libm::{sqrt, acos};

/// Quaternion: [w, x, y, z] — scalar first (aerospace convention)
/// Unit quaternion representing attitude/rotation.
pub type Quaternion = [f64; 4];

/// PID gains for attitude control (fixed, no scheduling).
///
/// Placeholder values. Real values require wind tunnel data or
/// CFD-derived aerodynamic coefficients [Ref 8, 13].
#[repr(C, align(16))]
pub struct PidGains {
    pub kp: f64,  // proportional gain
    pub ki: f64,  // integral gain
    pub kd: f64,  // derivative gain
}

/// Default PID gains — tuned for moderate hypersonic regime (Mach 5-10).
/// For Mach 15+, consider gain scheduling or adaptive control [Ref 5].
pub const DEFAULT_GAINS: PidGains = PidGains {
    kp: 12.0,
    ki: 0.8,
    kd: 4.5,
};

/// PID state — managed externally for stateless design.
///
/// Integral terms accumulate error over time. Must be reset
/// when mode changes or after large maneuvers to prevent windup.
#[repr(C, align(32))]
#[derive(Clone, Copy)]
pub struct PidState {
    /// Accumulated integral error for [roll, pitch, yaw] axes.
    pub integral: [f64; 3],
}

impl PidState {
    /// Create new zeroed PID state.
    #[inline(always)]
    pub const fn new() -> Self {
        Self {
            integral: [0.0; 3],
        }
    }

    /// Reset integral terms — call after mode change or large maneuver.
    #[inline(always)]
    pub fn reset(&mut self) {
        self.integral = [0.0; 3];
    }
}

/// Quaternion conjugate: q* = [w, -x, -y, -z]
///
/// For unit quaternions, conjugate equals inverse.
/// Used for computing relative rotation: q_err = q_target ⊗ q_current⁻¹
#[inline(always)]
pub fn quat_conjugate(q: &Quaternion) -> Quaternion {
    [q[0], -q[1], -q[2], -q[3]]
}

/// Quaternion multiplication (Hamilton product).
///
/// q₁ ⊗ q₂ = [w₁w₂ - v₁·v₂,  w₁v₂ + w₂v₁ + v₁×v₂]
///
/// Cost: 16 mul + 12 add (~30ns on aarch64)
#[inline(always)]
pub fn quat_multiply(q1: &Quaternion, q2: &Quaternion) -> Quaternion {
    let (w1, x1, y1, z1) = (q1[0], q1[1], q1[2], q1[3]);
    let (w2, x2, y2, z2) = (q2[0], q2[1], q2[2], q2[3]);

    [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,  // scalar
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,  // x
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,  // y
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,  // z
    ]
}

/// Convert quaternion to axis-angle representation.
///
/// Returns: (axis_x, axis_y, axis_z, angle_radians)
/// Axis is normalized. Angle in [0, π].
///
/// From quaternion q = [cos(θ/2),  sin(θ/2)·axis]:
///   angle = 2·acos(w)
///   axis = [x, y, z] / sin(θ/2)
///
/// Cost: 1 acos + 1 sqrt + normalization (~40ns)
#[inline(always)]
pub fn quat_to_axis_angle(q: &Quaternion) -> (f64, f64, f64, f64) {
    let w = q[0].clamp(-1.0, 1.0);  // protect against numerical drift
    let angle = 2.0 * acos(w);

    // Small angle: return zero axis to avoid division by zero
    if angle < 1e-12 {
        return (0.0, 0.0, 0.0, 0.0);
    }

    let sin_half = sqrt(1.0 - w * w);
    let scale = 1.0 / sin_half;

    (q[1] * scale, q[2] * scale, q[3] * scale, angle)
}

/// Compute PID output for each axis.
///
/// # Arguments
/// - `axis_angle`: (axis_x, axis_y, axis_z, angle) from quaternion error
/// - `rate`: angular rates [ω_x, ω_y, ω_z] in rad/s
/// - `gains`: PID gains
/// - `state`: integral state (updated in-place)
/// - `dt`: time step in seconds (for integral term)
///
/// # Returns
/// Control commands [u_roll, u_pitch, u_yaw]
///
/// # Control Law
/// u = Kp·(angle·axis) + Ki·∫(error) + Kd·(-rate)
///
/// D-term uses negative rate because rate is the derivative of attitude:
///   d(attitude)/dt = angular_rate
///   So derivative term = -rate for damping
#[inline(always)]
pub fn compute_pid(
    axis_angle: (f64, f64, f64, f64),
    rate: &[f64; 3],
    gains: &PidGains,
    state: &mut PidState,
    dt: f64,
) -> [f64; 3] {
    let (ax, ay, az, angle) = axis_angle;

    // Zero error: no correction needed
    if angle < 1e-12 {
        return [0.0, 0.0, 0.0];
    }

    // P-term: proportional to attitude error
    let p_term = angle;

    // D-term: derivative = -angular_rate (rate damping)
    let d_term = [-rate[0], -rate[1], -rate[2]];

    // I-term: accumulate error projected onto each axis
    const IMAX: f64 = 10.0;  // anti-windup limit
    state.integral[0] = (state.integral[0] + ax * angle * dt).clamp(-IMAX, IMAX);
    state.integral[1] = (state.integral[1] + ay * angle * dt).clamp(-IMAX, IMAX);
    state.integral[2] = (state.integral[2] + az * angle * dt).clamp(-IMAX, IMAX);

    // Output: u = Kp·P + Ki·I + Kd·D
    [
        gains.kp * ax * p_term + gains.ki * state.integral[0] + gains.kd * d_term[0],
        gains.kp * ay * p_term + gains.ki * state.integral[1] + gains.kd * d_term[1],
        gains.kp * az * p_term + gains.ki * state.integral[2] + gains.kd * d_term[2],
    ]
}

/// Allocate PID commands to control surfaces.
///
/// # Arguments
/// - `pid_output`: [u_roll, u_pitch, u_yaw] control commands
///
/// # Returns
/// Surface deflections [elevon_L, elevon_R, rudder, canard_L, canard_R, speed_brake]
/// in radians. Positive = trailing edge down.
///
/// # Allocation Matrix (Parker 2007, simplified)
/// - Elevons: pitch (negative) + roll (differential)
/// - Rudder: yaw only
/// - Canards: pitch (positive, opposite elevons)
/// - Speed brake: TODO drag management
///
/// # Limits
/// Elevons: ±25° (0.44 rad) — flow separation concern
/// Rudder:  ±20° (0.35 rad) — vertical stabilizer shadow
/// Canards: ±15° (0.26 rad) — forward fuselage interaction
/// Brake:    0..+60° — drag only, no negative
#[inline(always)]
pub fn allocate_surfaces(pid_output: [f64; 3]) -> [f64; 6] {
    let [u_roll, u_pitch, u_yaw] = pid_output;

    // Control allocation
    let elevon_l = -u_pitch + u_roll;
    let elevon_r = -u_pitch - u_roll;
    let rudder = u_yaw;
    let canard_l = u_pitch;
    let canard_r = u_pitch;
    let speed_brake = 0.0;  // TODO: drag management based on energy state

    let mut surfaces = [elevon_l, elevon_r, rudder, canard_l, canard_r, speed_brake];

    // Saturation limits
    const MAX_ELEVON: f64 = 0.436;  // ±25°
    const MAX_RUDDER: f64 = 0.349;  // ±20°
    const MAX_CANARD: f64 = 0.262;  // ±15°
    const MAX_BRAKE: f64 = 1.047;   // 0..+60°

    surfaces[0] = surfaces[0].clamp(-MAX_ELEVON, MAX_ELEVON);
    surfaces[1] = surfaces[1].clamp(-MAX_ELEVON, MAX_ELEVON);
    surfaces[2] = surfaces[2].clamp(-MAX_RUDDER, MAX_RUDDER);
    surfaces[3] = surfaces[3].clamp(-MAX_CANARD, MAX_CANARD);
    surfaces[4] = surfaces[4].clamp(-MAX_CANARD, MAX_CANARD);
    surfaces[5] = surfaces[5].clamp(0.0, MAX_BRAKE);

    surfaces
}

/// Compute attitude correction from current quaternion and angular rate.
///
/// Main entry point for stability control. Called each control cycle.
///
/// # Arguments
/// - `attitude_current`: Current vehicle attitude (quaternion)
/// - `attitude_target`: Desired attitude (quaternion)
/// - `angular_rate`: Current angular rates [rad/s]
/// - `gains`: PID gains
/// - `state`: PID state (updated in-place)
/// - `dt`: Control cycle time in seconds
///
/// # Returns
/// Surface deflections [elevon_L, elevon_R, rudder, canard_L, canard_R, speed_brake]
#[inline(always)]
pub fn compute_attitude_correction(
    attitude_current: &Quaternion,
    attitude_target: &Quaternion,
    angular_rate: &[f64; 3],
    gains: &PidGains,
    state: &mut PidState,
    dt: f64,
) -> [f64; 6] {
    // 1. Compute quaternion error: q_err = q_target ⊗ q_current⁻¹
    let q_current_inv = quat_conjugate(attitude_current);
    let q_err = quat_multiply(attitude_target, &q_current_inv);

    // 2. Extract axis-angle from error quaternion
    let axis_angle = quat_to_axis_angle(&q_err);

    // 3. PID computation
    let pid_output = compute_pid(axis_angle, angular_rate, gains, state, dt);

    // 4. Allocate to surfaces
    allocate_surfaces(pid_output)
}

/// Legacy compatibility — wrapper for old signature.
/// TODO: Remove once CentralProcessor is updated.
#[inline(always)]
pub fn compute_attitude_correction_legacy(
    attitude: &[f64; 4],
    angular_rate: &[f64; 3],
    deflections: &mut [f64; 6],
) {
    // Identity target (no rotation) + default gains + zero state
    let target = [1.0, 0.0, 0.0, 0.0];
    let gains = DEFAULT_GAINS;
    let mut state = PidState::new();
    let dt = 100e-9;  // 100ns control cycle

    let result = compute_attitude_correction(attitude, &target, angular_rate, &gains, &mut state, dt);
    deflections.copy_from_slice(&result);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quaternion_identity() {
        let id = [1.0, 0.0, 0.0, 0.0];
        let result = quat_multiply(&id, &id);
        assert!((result[0] - 1.0).abs() < 1e-12);
        assert!(result[1].abs() < 1e-12);
        assert!(result[2].abs() < 1e-12);
        assert!(result[3].abs() < 1e-12);
    }

    #[test]
    fn test_quaternion_conjugate() {
        let q = [0.5, 0.5, 0.5, 0.5];  // 120° about [1,1,1]
        let qc = quat_conjugate(&q);
        assert_eq!(qc[0], 0.5);
        assert_eq!(qc[1], -0.5);
        assert_eq!(qc[2], -0.5);
        assert_eq!(qc[3], -0.5);
    }

    #[test]
    fn test_quaternion_multiply_i_squared() {
        // i ⊗ i = -1  →  [0,1,0,0]² = [-1,0,0,0]
        let i = [0.0, 1.0, 0.0, 0.0];
        let result = quat_multiply(&i, &i);
        assert!((result[0] - (-1.0)).abs() < 1e-12);
        assert!(result[1].abs() < 1e-12);
        assert!(result[2].abs() < 1e-12);
        assert!(result[3].abs() < 1e-12);
    }

    #[test]
    fn test_quaternion_to_axis_angle_identity() {
        let id = [1.0, 0.0, 0.0, 0.0];  // zero rotation
        let (_x, _y, _z, angle) = quat_to_axis_angle(&id);
        assert!(angle < 1e-12);  // zero angle
    }

    #[test]
    fn test_quaternion_to_axis_angle_90_degrees() {
        // 90° about Z axis: [cos(45°), 0, 0, sin(45°)] = [0.707, 0, 0, 0.707]
        let q = [0.70710678, 0.0, 0.0, 0.70710678];
        let (x, y, z, angle) = quat_to_axis_angle(&q);
        assert!((angle - 1.57079633).abs() < 0.001);  // π/2
        assert!(x.abs() < 0.01);  // no X component
        assert!(y.abs() < 0.01);  // no Y component
        assert!((z - 1.0).abs() < 0.01);  // pure Z rotation
    }

    #[test]
    fn test_pid_zero_error() {
        let mut state = PidState::new();
        let gains = DEFAULT_GAINS;
        let axis_angle = (0.0, 0.0, 0.0, 0.0);
        let rate = [0.1, 0.2, 0.3];
        let result = compute_pid(axis_angle, &rate, &gains, &mut state, 1e-3);
        assert_eq!(result[0], 0.0);
        assert_eq!(result[1], 0.0);
        assert_eq!(result[2], 0.0);
    }

    #[test]
    fn test_allocate_surfaces_pitch_only() {
        let output = [0.0, 0.5, 0.0];  // pitch command only
        let surfaces = allocate_surfaces(output);
        assert!((surfaces[0] - (-0.5)).abs() < 0.01);  // elevon L
        assert!((surfaces[1] - (-0.5)).abs() < 0.01);  // elevon R
        assert!(surfaces[2].abs() < 0.01);  // rudder
        assert!((surfaces[3] - 0.5).abs() < 0.01);  // canard L
        assert!((surfaces[4] - 0.5).abs() < 0.01);  // canard R
    }

    #[test]
    fn test_allocate_surfaces_roll_only() {
        let output = [0.3, 0.0, 0.0];  // roll command only
        let surfaces = allocate_surfaces(output);
        assert!((surfaces[0] - 0.3).abs() < 0.01);  // elevon L positive
        assert!((surfaces[1] - (-0.3)).abs() < 0.01);  // elevon R negative
        assert!(surfaces[2].abs() < 0.01);  // rudder
        assert!(surfaces[3].abs() < 0.01);  // canard L
        assert!(surfaces[4].abs() < 0.01);  // canard R
    }

    #[test]
    fn test_allocate_surfaces_saturation() {
        let output = [10.0, 10.0, 10.0];  // huge command
        let surfaces = allocate_surfaces(output);
        assert!(surfaces[0].abs() <= 0.44);  // elevon limit
        assert!(surfaces[1].abs() <= 0.44);
        assert!(surfaces[2].abs() <= 0.35);  // rudder limit
        assert!(surfaces[3].abs() <= 0.26);  // canard limit
        assert!(surfaces[4].abs() <= 0.26);
    }
}
