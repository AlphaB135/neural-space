# References

## Aerodynamics & Shock Physics

1. **Billig, F.S.** (1967). "Shock-Wave Shapes Around Spherical- and Cylindrical-Nosed Bodies."
   *Journal of Spacecraft and Rockets*, 4(6), 822–823.
   — Shock standoff distance correlation used in `central/propulsion.rs`.
   Formula: Δ/Rₙ = 0.143 · exp(3.24/M²) for spheres.

2. **Billig, F.S.** (1969). "Shock Standoff Distance for Spherical-Nosed Bodies."
   *Journal of Spacecraft and Rockets*.
   — Extension of the 1967 correlation.

3. **Anderson, J.D.** (2019). *Hypersonic and High-Temperature Gas Dynamics* (3rd ed.).
   AIAA Education Series.
   — Foundational text for real gas effects, dissociation, and ionization at Mach 20+.
   Billig correlation assumes calorically perfect gas (γ=1.4); real gas corrections needed above Mach 15.

4. **Wen, C.-Y. & Hornung, H.G.** (1995). "Non-equilibrium dissociating flow over spheres."
   *Journal of Fluid Mechanics*, 299, 389–405.
   — Real gas effect corrections to shock standoff for high-enthalpy flows.

## Control Systems

5. **Dong, C., Xu, L., Chen, Y., & Zhang, H.** (2022). "Finite-Time Deterministic Learning Command Filtered Control for Hypersonic Flight Vehicle."
   *ResearchGate*. [Link](https://www.researchgate.net/publication/359345073)
   — Deterministic learning theory with finite-time convergence guarantees.
   Relevant to `central/stability.rs` PID gain scheduling approach.

6. **Bu, X., Wu, X., Zhu, R., & Ma, J.** (2021). "Adaptive control of hypersonic vehicles with unknown dynamics."
   *Acta Astronautica*. [DOI](https://doi.org/10.1016/j.actaastro.2021.09.032)
   — Dual neural network adaptive control for robust tracking.

7. **Shen, Q., Jiang, B., & Cocquempot, V.** (2024). "Fault-Tolerant Tracking Control of Hypersonic Vehicle."
   *Drones*, 8(7), 295. [DOI](https://doi.org/10.3390/drones8070295)
   — Prescribed error bounds with actuator fault tolerance.
   Relevant to `reflex/` fault detection thresholds.

8. **Parker, J.T., Serrani, A., Yurkovich, S., Bolender, M.A., & Doman, D.B.** (2007).
   "Control-oriented modeling of an air-breathing hypersonic vehicle."
   *Journal of Guidance, Control, and Dynamics*, 30(3), 856–869.
   — Standard reference for hypersonic vehicle control modeling.

## Thermal Protection & Boundary Layer

9. **Li, X. et al.** (2024). "Controlling Hypersonic Boundary Layer Transition with Localized Cooling & Metasurface Treatment."
   *Scientific Reports*. [DOI](https://doi.org/10.1038/s41598-024-66867-4)
   — Novel transition control via wall cooling and metasurface treatment.

10. **Zhang, Y. et al.** (2025). "Thermal Protection Systems for Hypersonic and Re-Entry Vehicles."
    *ScienceDirect*. [DOI](https://doi.org/10.1016/j.matpr.2025.01.832)
    — Comprehensive TPS review: thermal barrier, aerodynamic surface, shock absorption.

11. **Berridge, D.C. et al.** (2025). "Experimental Visualization of Hypersonic Boundary Layer Transition."
    *AIAA*. [DOI](https://doi.org/10.2514/6.2025-0558)
    — Mack second mode visualization and surface heat transfer measurement.

12. **Zhu, W. et al.** (2022). "Research Progress of Hypersonic Boundary Layer Transition Control."
    *Advances in Aerodynamics*. [Open Access](https://doi.org/10.1186/s42774-022-00105-1)
    — Review of transition control methods to reduce TPS burnout risk.

## Foundational

13. **Fidan, B., Mirmirani, M., & Ioannou, P.** (2003). "Flight Dynamics and Control of Air-Breathing Hypersonic Vehicles: Review and New Directions."
    *AIAA International Space Planes and Hypersonic Systems and Technologies*.
    — Standard survey of hypersonic flight dynamics challenges.

14. **DTIC** (2010). "On Stability and Control of Hypersonic Vehicles."
    [PDF](https://apps.dtic.mil/sti/pdfs/ADA521248.pdf)
    — DoD foundational report on aerodynamic characteristics, stability, and control issues.

15. **University of Notre Dame Hypersonics Initiative.** "Flight Control, Guidance, and Navigation."
    [Link](https://hypersonics.nd.edu/research/flight-control-guidance-and-navigation/)
    — Ongoing research on adaptive and robust control for hypersonic systems.

---

## Citation Policy

Code that implements a specific formula or algorithm from a paper MUST include:
1. A comment referencing the paper number (e.g., `[Ref 1]`)
2. The original author and year
3. Any modifications or approximations made

Example:
```rust
// Billig shock standoff correlation [Ref 1]
// Modified: using f64 instead of lookup table for embedded target
pub fn standoff_distance(mach: f64, nose_radius: f64) -> f64 {
    0.143 * nose_radius * (3.24 / (mach * mach)).exp()
}
```
