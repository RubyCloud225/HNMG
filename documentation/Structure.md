# HNMG Architecture Documentation
## Hamiltonian Natural Motion Generation

*Catherine Earl — May 2026*

---

## Overview

HNMG generates physically natural motion for robotic systems by grounding trajectory
planning in Hamiltonian mechanics. Rather than prescribing trajectories geometrically,
the system lets the physics generate motion naturally — the robot moves along paths
determined by its own mechanical Hamiltonian, minimising action and producing
inherently smooth, energy-efficient motion.

The framework is structured across five components, each mapping onto a distinct
physical or mathematical concept.

---

## Class Structure

```
config.h                    — compile-time constants
robot_dynamics.h            — Part 2: RobotDynamics
liouville_operator.h        — Part 3: LiouvilleOperator + ActionFunctional
natural_mode_evolution.h    — Part 4: NaturalModeEvolution
c_space.cpp                 — Parts 1 & 5: PhaseSpacePoint, PhaseSpaceDensity,
                              CSpaceEvolution, main()
```

---

## Part 2: RobotDynamics

The full robot equation of motion in 30-dimensional configuration space:

```
M(q)q̈ = τ - C(q,q̇)q̇ - G(q)
```

Reformulated as Hamilton's equations in phase space (q, p):

```
q̇ = ∂H/∂p = M(q)⁻¹ p
ṗ = -∂H/∂q = -G(q) - C(q,q̇)q̇
```

Newton's third law is embedded in the off-diagonal entries of M(q) — the
action-reaction coupling between joints propagating through the kinematic chain.

---

## Part 3: Two Classes

Part 3 splits into two classes with distinct responsibilities.

### LiouvilleOperator

Integrates phase space forward in time. Preserves phase space volume.

The Liouville equation governs how a phase space density evolves:

```
∂ρ/∂t = {H, ρ}    (Poisson bracket)
```

The stochastic extension is the Langevin equation:

```
dq = (∂H/∂p) dt + √(2α) dWq
dp = -(∂H/∂q) dt + √(2α) dWp
```

Deterministic drift follows Hamilton's equations. The diffusion term handles
model uncertainty and natural variation in motion.

**Symplectic integration — why order matters:**

```
Symplectic Euler (correct):
  p_new = p - ∇V * dt         update momentum first
  q_new = q + M⁻¹p_new * dt   use updated momentum

Standard Euler (incorrect):
  energy drifts — physically inconsistent trajectories
```

### ActionFunctional

Scores completed trajectories. Ranks candidates. Selects the most natural motion.

```
S[q] = ∫₀ᵀ L(q, q̇) dt = ∫₀ᵀ (T - V) dt
```

By Hamilton's Principle, the trajectory a physical system actually takes is the
one for which δS = 0. Natural motion minimises action. This is not an
approximation — it is equivalent to Newton's laws exactly.

The beta schedule controls the balance between deterministic Hamiltonian flow
and stochastic exploration over the planning horizon:

```
β_t = β₀(1 - t/T) + t/T

t small → trust physics, follow Hamiltonian flow
t large → explore, find goal region
```

---

## Part 4: NaturalModeEvolution

Decomposes the inertia matrix into natural motion modes. Propagates each mode
independently. Reconstructs in joint space.

```
M(q) = V Λ Vᵀ

V = eigenvectors — natural motion directions
Λ = eigenvalues  — natural frequencies
```

Small eigenvalue → energetically cheap direction of motion
Large eigenvalue → energetically costly direction of motion

The propagator evolves the robot state along natural modes:

```
q(t+dt) = V exp(-Λdt) Vᵀ q(t)
```

An adaptive diffusion coefficient α_t modulates exploration based on proximity
to the goal and confidence in the local dynamics model.

---

## Part 5: CSpaceEvolution

Combines all components. Runs the full planning loop.

```
1. Initialise PhaseSpaceDensity as Gaussian around q_start

2. For each candidate trajectory:
   a. Sample initial (q, p) from density
   b. Evolve via LiouvilleOperator
   c. Store trajectory

3. Score all trajectories via ActionFunctional

4. Importance resample — weight particles by goal proximity
   w_i ∝ exp(-||q_final - q_goal||² / σ²)

5. Return minimum action trajectory
```

---

## Validation Checkpoints

| Checkpoint | What to check | Expected behaviour |
|---|---|---|
| Part 2 | H(q,p) over time, no control | Energy oscillates, does not drift |
| Part 3 | Phase space volume | Symplectic structure preserved |
| Part 3 | Ehrenfest theorem | Mean state follows Hamilton's equations |
| Part 4 | Modal reconstruction | V Vᵀ q = q to machine precision |
| Part 5 | Action ranking | Lower action trajectories visually smoother |
| Part 5 | Goal convergence | Mean configuration reaches q_goal |

---

## File Dependencies

```
config.h
    └── robot_dynamics.h
    └── liouville_operator.h
            └── robot_dynamics.h
    └── natural_mode_evolution.h
            └── robot_dynamics.h
            └── liouville_operator.h
    └── c_space.cpp
            └── robot_dynamics.h
            └── liouville_operator.h
            └── natural_mode_evolution.h
```

---

*Catherine Earl — May 2026*
*HNMG: Hamiltonian Natural Motion Generation*