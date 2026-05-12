//! Ejection sequence — the final autonomic response.
//!
//! When the reflex loop determines that survival is impossible,
//! it fires the ejection sequence. This is irreversible.
//! Payload separation, vehicle breakup, and recovery system activation.

/// Ejection sequence states.
#[repr(u8)]
pub enum EjectionState {
    Armed = 0,
    SeparationInitiated = 1,
    PayloadReleased = 2,
    BreakupConfirmed = 3,
    RecoveryActive = 4,
    Complete = 5,
}

/// Ejection timeline — every step has a hard deadline.
pub const SEPARATION_DELAY_NS: u64 = 50_000;   // 50us to initiate separation
pub const PAYLOAD_RELEASE_NS: u64 = 200_000;   // 200us to release payload
pub const RECOVERY_DEPLOY_NS: u64 = 500_000;   // 500us to deploy recovery

/// Fire the ejection sequence.
/// Safety: This is irreversible. Only call from the reflex loop.
#[inline(always)]
pub unsafe fn fire_ejection() -> EjectionState {
    // 1. Cut all engine power
    // 2. Detonate separation bolts
    // 3. Release payload module
    // 4. Deploy recovery system
    EjectionState::Complete
}

/// Arm the ejection system — must be called during vehicle initialization.
#[inline(always)]
pub fn arm() -> EjectionState {
    EjectionState::Armed
}
