# HNMG — Hamiltonian Natural Motion Generation

**Author:** Catherine Earl  
**Date:** May 2026  
**Status:** Active Development  
**Language:** C++ (Eigen, Jacobi)

---

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial 4.0 International License (CC BY-NC 4.0)**.

You are free to share and adapt this work for non-commercial purposes, provided appropriate credit is given. Commercial use of this work, in whole or in part, requires explicit written permission from the author.

> © 2026 Catherine Earl. All rights reserved for commercial use.  
> The phase space diffusion methodology in this project derives from independent prior research in quantum hybrid inference conducted under OmniHenos Ltd (2023–present). Unauthorised commercial appropriation of this framework, its architecture, or its methodology will be pursued.

[![License: CC BY-NC 4.0](https://img.shields.io/badge/License-CC%20BY--NC%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc/4.0/)

---

## Overview

HNMG generates physically natural motion for robotic and humanoid systems by grounding trajectory planning directly in Hamiltonian mechanics. Rather than prescribing trajectories geometrically — splines, waypoints, RRT — the system lets the physics generate motion naturally.

The robot moves along paths determined by its own mechanical Hamiltonian, minimising action and producing inherently smooth, energy-efficient motion. This is not an approximation of natural movement. It is the same physical principle governing natural movement.

The framework transfers methodology from quantum-theoretic phase space diffusion into classical robotics — specifically, the application of Hamiltonian state evolution, symplectic integration, and adaptive diffusion schedules to configuration space trajectory planning.

---

## Motivation

Standard trajectory planning methods share a common limitation: they impose geometry on a system that is fundamentally physical. Cubic splines ignore dynamics. RRT produces paths that require post-hoc smoothing. CHOMP minimises a cost function that approximates but does not derive from the underlying physics.

Human arm motion is Hamiltonian. Studies of human reaching movements show trajectories that approximately minimise action — the same criterion this framework uses. That smoothness and efficiency is not prescribed. It emerges from the physics.

HNMG generates motion by the same principle. Naturalness is not a post-processing step. It is the output of the planning algorithm by construction.

---

## Theoretical Foundation

### Hamiltonian Mechanics in Configuration Space

A robot with n joints has a configuration space (C-space) of dimension n. Every point in C-space is a complete description of joint state. Motion is a curve q(t) through this space.

The robot Hamiltonian is the total mechanical energy expressed in phase space (q, p):

```
H(q, p) = ½ pᵀ M(q)⁻¹ p + V(q)
```

Where:
- `q` — joint configuration vector
- `p = M(q)q̇` — generalised momentum (inertia-weighted joint velocity)
- `M(q)` — configuration-dependent inertia matrix
- `V(q)` — potential energy (gravity, conservative forces)

Motion follows Hamilton's equations:

```
q̇ =  ∂H/∂p = M(q)⁻¹ p
ṗ = -∂H/∂q = -∇V(q) + ...
```

This is the same mathematical structure as the quantum Hamiltonian — state evolves under a Hamiltonian operator. The difference is classical phase space point versus quantum state vector. The formalism transfers directly.

---

### Principle of Least Action

The trajectory a physical system takes between two states is the one for which the action S is stationary:

```
S[q] = ∫₀ᵀ L(q, q̇) dt = ∫₀ᵀ (T - V) dt
```

This is Hamilton's Principle. It is not an approximation. It is exactly equivalent to Newton's laws and gives the same equations of motion.

Natural robot motion — the trajectory that requires minimum energy expenditure — minimises action. HNMG uses action as the ranking criterion across candidate trajectories. The most physically natural trajectory is selected by the same principle nature uses.

---

### Natural Mode Decomposition via Jacobi Eigendecomposition

The inertia matrix M(q) is symmetric positive definite. Jacobi eigendecomposition yields:

```
M(q) = V Λ Vᵀ
```

Where:
- `V` — eigenvectors: natural motion directions at the current configuration
- `Λ` — eigenvalues: natural frequencies (cost of motion in each mode)

Small eigenvalue → energetically cheap direction  
Large eigenvalue → energetically costly direction

The natural mode propagator evolves state along these directions independently:

```
q(t+dt) = V exp(-Λdt) Vᵀ q(t)
```

This is structurally identical to the quantum propagator `U = V exp(-iλdt) V†`. The eigendecomposition finds the natural basis. The exponential propagates along natural modes. Back-transformation returns to joint space. The mathematical transfer from quantum to classical is exact.

---

### Phase Space Diffusion

Rather than planning a single deterministic trajectory, HNMG treats motion generation as a diffusion process in phase space (q, p):

```
dq = (∂H/∂p) dt + √(2α) dWq
dp = -(∂H/∂q) dt + √(2α) dWp
```

Deterministic drift follows Hamilton's equations. The stochastic term handles model uncertainty, external perturbation, and path exploration. This is the Langevin equation applied to robot phase space — methodology transferred directly from quantum hybrid inference research.

The diffusion coefficient α is adaptive, modulated by a beta schedule over the planning horizon:

```
β_t = β₀(1 - t/T) + t/T

t small → trust physics, follow Hamiltonian flow  
t large → explore, find goal region
```

---

### Symplectic Integration

Phase space volume must be preserved — this is Liouville's theorem and defines what makes Hamiltonian mechanics physically consistent.

Standard Euler integration does not preserve this structure. Energy drifts. Trajectories become physically inconsistent over time.

HNMG uses Symplectic Euler integration, which preserves the symplectic structure exactly:

```
p_new = p - ∇V · dt          (momentum updated first)
q_new = q + M⁻¹p_new · dt    (position uses updated momentum)
```

Order matters. This is not a minor implementation detail — it is what guarantees physically valid trajectories over long planning horizons.

---

## Architecture

```
Input: Current state (q₀, p₀) + Goal configuration q_goal
          ↓
Step 1: Jacobi Diagonalisation
        M(q) = V Λ Vᵀ → natural modes V, frequencies Λ
          ↓
Step 2: Phase Space Diffusion
        Symplectic Euler steps with adaptive beta schedule
        Multiple candidate trajectories sampled
          ↓
Step 3: Action-Based Ranking
        Score each trajectory: S = ∫L dt
        Select minimum action trajectory
          ↓
Step 4: Natural Mode Propagation
        q(t+dt) = V exp(-Λdt) Vᵀ q(t)
        Physically natural, smooth motion
          ↓
Step 5: Task Space Projection (when required)
        ẋ = J(q) q̇ → end effector tracking
          ↓
Output: Joint torques τ = M(q)q̈ + C(q,q̇)q̇ + G(q)
```

---

## Class Structure

```
config.h                    — compile-time constants, joint limits
robot_dynamics.h            — RobotDynamics: inertia, Coriolis, gravity
liouville_operator.h        — LiouvilleOperator, ActionFunctional
natural_mode_evolution.h    — NaturalModeEvolution: Jacobi decomposition,
                              modal propagation
c_space.cpp                 — PhaseSpacePoint, PhaseSpaceDistribution,
                              CSpaceEvolution, main()
```

---

## Key Dependencies

| Library | Purpose |
|---|---|
| Eigen | Linear algebra, matrix operations, eigendecomposition |
| Jacobi (via Eigen SelfAdjointEigenSolver) | Natural mode decomposition of inertia matrix |

Zero external robotics framework dependencies. Dynamics are implemented from first principles.

---

## What Makes This Different

| Method | Basis | Limitation |
|---|---|---|
| Cubic splines | Geometric interpolation | Ignores dynamics |
| RRT / RRT* | Random sampling | Jerky, requires smoothing |
| CHOMP | Gradient descent on cost | Local minima, cost function is approximate |
| MPC | Receding horizon optimisation | Linearisation errors, computationally expensive |
| **HNMG** | **Hamiltonian mechanics + least action** | **Physics-grounded by construction** |

---

## Transferable Methodology

| HNMG (Robotics) | OmniHenos MAHQ (Quantum Hybrid Inference) |
|---|---|
| Phase space (q, p) diffusion | Phase space diffusion engine |
| Symplectic Euler integration | Symplectic propagator for quantum state |
| Hamiltonian evolution H(q,p) | Hamiltonian encoder H(t) |
| Natural mode propagator V exp(-Λdt) Vᵀ | Quantum propagator V exp(-iλdt) V† |
| Action functional S = ∫L dt | KL divergence minimisation |
| Jacobi eigendecomposition | Mirror ancilla projection |

The mathematical framework is consistent across both domains. HNMG demonstrates that the physics-based methodology developed for quantum hybrid inference generalises to classical robotics trajectory planning.

---

## Development Status

HNMG is in active development. Current build status:

- [x] Theoretical framework complete
- [x] Architecture designed and documented
- [x] Hamiltonian dynamics formulation
- [x] Jacobi natural mode decomposition
- [x] Symplectic Euler integrator
- [ ] Full C-space evolution pipeline (in progress)
- [ ] Adaptive beta schedule integration (in progress)
- [ ] Action-based trajectory ranking (in progress)
- [ ] Validation suite against benchmark trajectories (planned)
- [ ] Task space projection layer (planned)

---

## Attribution

If you reference this work in research or publications:

```
Earl, C. (2026). HNMG: Hamiltonian Natural Motion Generation via 
Phase Space Diffusion and Least Action Principles. 
GitHub: github.com/RubyCloud225/HNMG
```

---

## Contact

Catherine Earl  
catherineearl8@gmail.com  
GitHub: RubyCloud225  
LinkedIn: Catherine Earl

---

*This repository represents original independent research. The phase space diffusion 
methodology transfers directly from prior work in quantum hybrid inference (OmniHenos Ltd, 
2023–present). Timestamped authorship trail established May 2026.*