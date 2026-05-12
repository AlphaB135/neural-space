//! Central Processing — The Brain
//!
//! Deterministic matrix math for stability and propulsion control.
//! Calculates optimal thrust vectoring and fuel mixture to maintain
//! the Shockwave Stand-off without melting the engine.

pub mod stability;
pub mod propulsion;
pub mod matrix;

/// Central processing tick — one complete control cycle.
pub const CONTROL_CYCLE_NS: u64 = 100; // 100ns target cycle time

/// Vehicle state vector — the complete snapshot of "what is happening now".
#[repr(C, align(32))]
pub struct VehicleState {
    pub position: [f64; 3],        // ECI frame, meters
    pub velocity: [f64; 3],        // m/s
    pub attitude: [f64; 4],        // quaternion
    pub angular_rate: [f64; 3],    // rad/s
    pub mass: f64,                 // kg (changes as fuel burns)
    pub mach: f64,
    pub dynamic_pressure: f64,     // Pa
    pub skin_temp_max: f64,        // K
    pub elapsed_ns: u64,
}

/// Control output — what the brain decided to do.
#[repr(C, align(16))]
pub struct ControlOutput {
    pub thrust_vector: [f64; 3],    // desired thrust direction (unit)
    pub throttle: f64,              // 0.0–1.0
    pub fuel_mixture: f64,          // oxidizer/fuel ratio
    pub surface_deflections: [f64; 6], // control surface angles, radians
    pub cycle_time_ns: u64,         // how long this computation took
}

/// The brain — processes sensor input into control output in bounded time.
pub struct CentralProcessor {
    state: VehicleState,
    output: ControlOutput,
}

impl CentralProcessor {
    pub const fn new() -> Self {
        Self {
            state: VehicleState {
                position: [0.0; 3],
                velocity: [0.0; 3],
                attitude: [1.0, 0.0, 0.0, 0.0],
                angular_rate: [0.0; 3],
                mass: 0.0,
                mach: 0.0,
                dynamic_pressure: 0.0,
                skin_temp_max: 0.0,
                elapsed_ns: 0,
            },
            output: ControlOutput {
                thrust_vector: [0.0, 0.0, 1.0],
                throttle: 0.0,
                fuel_mixture: 2.56, // LOX/LH2 stoichiometric
                surface_deflections: [0.0; 6],
                cycle_time_ns: 0,
            },
        }
    }

    /// Process one control cycle — strictly bounded execution time.
    pub fn process(&mut self, state: VehicleState) -> &ControlOutput {
        self.state = state;

        // Stability computation
        stability::compute_attitude_correction(
            &self.state.attitude,
            &self.state.angular_rate,
            &mut self.output.surface_deflections,
        );

        // Propulsion computation
        propulsion::compute_thrust_profile(
            self.state.mach,
            self.state.dynamic_pressure,
            self.state.skin_temp_max,
            &mut self.output.thrust_vector,
            &mut self.output.throttle,
            &mut self.output.fuel_mixture,
        );

        &self.output
    }
}
