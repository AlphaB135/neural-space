# neural-space

A Deterministic Autonomic Nervous System for Hypersonic Aerospace Vehicles.

## What the hell is this?

This is **neural-space**. It is not another bloated, generic flight controller written by script kiddies who think a 10ms latency is "real-time". It is a synthetic biological nervous system designed for one thing: keeping a hypersonic vehicle from vaporizing itself and its payload at Mach 32.

If you are looking for a plug-and-play drone controller, leave. This is bare-metal, strictly deterministic Rust code built to survive extreme G-forces, thermal dissociation, and structural nightmares.

## Why?

Because human reaction time is garbage, and traditional state machines are too slow. At 40,000 km/h, your environment changes entirely in a fraction of a millisecond. If your system has to wait for a garbage collector or a bloated abstraction layer to figure out that the skin temperature is exceeding 10,000 degrees, you are already dead.

neural-space mimics a biological nervous system. It does not just "read sensors"; it **feels** the structural strain, anticipates the boundary layer collapse, and reacts via autonomic reflexes before the main logic loop even registers the pain.

## Architecture

The system is strictly divided into biological equivalents:

### Afferent Subsystem (Sensory Input)
Direct Memory Access (DMA) polling of thermal, pressure, and structural strain gauges. No buffering. No middleware. We read the metal directly.

### Central Processing (The Brain)
Deterministic matrix math for stability and propulsion control. It calculates the optimal thrust vectoring and fuel mixture to maintain the Shockwave Stand-off without melting the engine.

### Efferent Subsystem (Actuation/Muscles)
Sub-millisecond hardware manipulation. It fires the liquid rocket actuators and control surfaces exactly when needed.

### Spinal Reflex Loop (The "Oh Shit" Protocol)
Hardware-level interrupts. If structural failure is imminent or a hard start occurs in the turbopump, this loop bypasses the brain entirely. It cuts the fuel and triggers the ejection sequence in microseconds. It does not ask for permission.

```
src/
├── lib.rs              # no_std entry, panic handler
├── afferent/           # Sensory Input
│   ├── mod.rs          # DMA polling bus
│   ├── thermal.rs      # Skin temperature monitoring
│   ├── pressure.rs     # Stagnation/static pressure
│   └── strain.rs       # Structural strain gauges
├── central/            # The Brain
│   ├── mod.rs          # VehicleState, ControlOutput, control cycle
│   ├── stability.rs    # PID attitude control
│   ├── propulsion.rs   # Thrust vectoring, shock stand-off
│   └── matrix.rs       # Deterministic 3x3/4x4 matrix ops
├── efferent/           # Muscles
│   ├── mod.rs          # Efferent command bus
│   ├── actuators.rs    # Rocket engine valves, gimbal
│   └── surfaces.rs     # Elevons, rudder, canards
└── reflex/             # Oh Shit Protocol
    ├── mod.rs          # Reflex loop, priority checks
    ├── interrupt.rs    # Hardware interrupt handlers
    └── ejection.rs     # Irreversible ejection sequence
```

## Building and Running

Do not ask me to support stable Rust. We use Nightly because we need raw performance features that aren't finalized yet.

```bash
git clone https://github.com/AlphaB135/neural-space.git
cd neural-space
cargo +nightly build --release --target=aarch64-unknown-none
```

You need custom hardware to run this. If you try to run this on a Raspberry Pi, I will personally laugh at your funeral.

## Contributing

Send a pull request if you have a patch that reduces latency or optimizes memory alignment. If you send me a PR to "fix typos" or "add a code of conduct", I will block you. We care about physics and surviving Mach 32, not your feelings.

## Disclaimer

This software is designed to push the limits of atmospheric flight. It assumes your aerodynamic math is perfect and your materials can withstand hell. If you die using this code, it means your hardware was weak or your physics were wrong. Do not blame my code.
