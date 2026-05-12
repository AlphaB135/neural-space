//! Ejection sequence — the final autonomic response.
//!
//! When the reflex loop determines that survival is impossible,
//! it fires the ejection sequence. This is irreversible.
//! Payload separation, vehicle breakup, and recovery system activation.

/// Ejection sequence states.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq)]
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
///
/// # Safety
/// This function performs unsafe operations:
/// - Writes to hardware MMIO registers (explosive bolts, pyrotechnics)
/// - Never returns (vehicle breakup imminent)
///
/// Callers MUST ensure this is only executed when ejection is truly necessary.
#[inline(always)]
pub unsafe fn fire_ejection() -> EjectionState {
    // 1. Cut all engine power (write to fuel valve register)
    // TODO: Implement MMIO write to fuel controller

    // 2. Detonate separation bolts (write to pyro controller)
    // TODO: Implement MMIO write to pyro initiator

    // 3. Release payload module (mechanical latch release)
    // TODO: Implement MMIO write to latch controller

    // 4. Deploy recovery system (parachute/retro-rocket)
    // TODO: Implement MMIO write to recovery deployer

    // In a real implementation, this would trigger hardware actions
    // and never return. For now, return Complete state.
    EjectionState::Complete
}

/// Arm the ejection system — must be called during vehicle initialization.
///
/// Enables the pyrotechnic initiators and performs continuity checks
/// on all explosive bolts and separation systems.
#[inline(always)]
pub fn arm_ejection() -> EjectionState {
    // TODO: Implement MMIO writes to:
    // - Arm pyro controller
    // - Check continuity on separation bolts
    // - Verify recovery system integrity

    EjectionState::Armed
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_arm_ejection() {
        let state = arm_ejection();
        assert_eq!(state, EjectionState::Armed);
    }

    #[test]
    fn test_fire_ejection() {
        let state = unsafe { fire_ejection() };
        assert_eq!(state, EjectionState::Complete);
    }

    #[test]
    fn test_state_ord() {
        assert!(EjectionState::Armed < EjectionState::SeparationInitiated);
        assert!(EjectionState::SeparationInitiated < EjectionState::PayloadReleased);
        assert!(EjectionState::PayloadReleased < EjectionState::BreakupConfirmed);
        assert!(EjectionState::BreakupConfirmed < EjectionState::RecoveryActive);
        assert!(EjectionState::RecoveryActive < EjectionState::Complete);
    }
}
