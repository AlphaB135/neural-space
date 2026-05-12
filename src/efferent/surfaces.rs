//! Control surface actuation.
//!
//! Elevons, rudder, canards, and speed brake. At hypersonic speeds,
//! control surfaces must move in micro-radian increments. A single
//! degree of error creates tons of unwanted lift or drag.

use super::EfferentBus;

/// Surface register addresses (MMIO).
pub const ELEVON_PORT: u16 = 0x0040;
pub const ELEVON_STARBOARD: u16 = 0x0041;
pub const RUDDER: u16 = 0x0042;
pub const CANARD_PORT: u16 = 0x0043;
pub const CANARD_STARBOARD: u16 = 0x0044;
pub const SPEED_BRAKE: u16 = 0x0045;

/// Maximum deflection in radians.
pub const MAX_DEFLECTION: f64 = 0.2618; // ~15 degrees

/// Set all control surface deflections at once.
#[inline(always)]
pub fn set_deflections(
    bus: &mut EfferentBus,
    elevon_p: f64,
    elevon_s: f64,
    rudder: f64,
    canard_p: f64,
    canard_s: f64,
    speed_brake: f64,
    deadline_ns: u64,
) {
    bus.enqueue(ELEVON_PORT, elevon_p.clamp(-MAX_DEFLECTION, MAX_DEFLECTION), deadline_ns);
    bus.enqueue(ELEVON_STARBOARD, elevon_s.clamp(-MAX_DEFLECTION, MAX_DEFLECTION), deadline_ns);
    bus.enqueue(RUDDER, rudder.clamp(-MAX_DEFLECTION, MAX_DEFLECTION), deadline_ns);
    bus.enqueue(CANARD_PORT, canard_p.clamp(-MAX_DEFLECTION, MAX_DEFLECTION), deadline_ns);
    bus.enqueue(CANARD_STARBOARD, canard_s.clamp(-MAX_DEFLECTION, MAX_DEFLECTION), deadline_ns);
    bus.enqueue(SPEED_BRAKE, speed_brake.clamp(0.0, MAX_DEFLECTION), deadline_ns);
}
