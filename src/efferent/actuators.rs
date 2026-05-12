//! Liquid rocket actuator control.
//!
//! Controls main engine valves, turbopump throttle, and gimbal actuators.
//! These are the primary "muscles" of the vehicle.

use super::EfferentBus;

/// Actuator register addresses (MMIO).
pub const MAIN_ENGINE_VALVE: u16 = 0x0010;
pub const TURBOPUMP_THROTTLE: u16 = 0x0011;
pub const GIMBAL_PITCH: u16 = 0x0020;
pub const GIMBAL_YAW: u16 = 0x0021;
pub const FUEL_INJECTOR: u16 = 0x0030;
pub const OXIDIZER_VALVE: u16 = 0x0031;

/// Set throttle position (0.0–1.0).
#[inline(always)]
pub fn set_throttle(bus: &mut EfferentBus, throttle: f64, deadline_ns: u64) {
    bus.enqueue(TURBOPUMP_THROTTLE, throttle.clamp(0.0, 1.0), deadline_ns);
    bus.enqueue(MAIN_ENGINE_VALVE, if throttle > 0.01 { 1.0 } else { 0.0 }, deadline_ns);
}

/// Set gimbal angles for thrust vectoring.
#[inline(always)]
pub fn set_gimbal(bus: &mut EfferentBus, pitch: f64, yaw: f64, deadline_ns: u64) {
    bus.enqueue(GIMBAL_PITCH, pitch.clamp(-0.15, 0.15), deadline_ns);
    bus.enqueue(GIMBAL_YAW, yaw.clamp(-0.15, 0.15), deadline_ns);
}

/// Emergency shutdown — close all valves immediately.
#[inline(always)]
pub fn emergency_shutdown(bus: &mut EfferentBus, deadline_ns: u64) {
    bus.enqueue(MAIN_ENGINE_VALVE, 0.0, deadline_ns);
    bus.enqueue(FUEL_INJECTOR, 0.0, deadline_ns);
    bus.enqueue(OXIDIZER_VALVE, 0.0, deadline_ns);
    bus.enqueue(TURBOPUMP_THROTTLE, 0.0, deadline_ns);
}
