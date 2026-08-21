#pragma once

/**
 * @file liouville_operator.h
 * @author Catherine Earl
 * @brief Part 3A — LiouvilleOperator and Part 3B — ActionFunctional
 *
 * Part 3A: LiouvilleOperator
 * --------------------------
 * The Liouville equation governs how a phase space density evolves over time:
 *
 *   ∂ρ/∂t = {H, ρ}
 *
 * Where {H, ρ} is the Poisson bracket:
 *
 *   {H, ρ} = ∂H/∂p · ∂ρ/∂q - ∂H/∂q · ∂ρ/∂p
 *
 * This is the classical equation of motion for a probability density in phase
 * space. A single particle trajectory is a special case — a delta function
 * density that moves under Hamilton's equations.
 *
 * The stochastic extension (Langevin equation) adds controlled noise:
 *
 *   dq = (∂H/∂p) dt + √(2α) dWq
 *   dp = -(∂H/∂q) dt + √(2α) dWp
 *
 * The deterministic drift follows Hamilton's equations exactly.
 * The diffusion term √(2α) dW handles model uncertainty, external
 * perturbations, and natural variation — producing a distribution of
 * physically valid trajectories rather than a single deterministic path.
 *
 * Part 3B: ActionFunctional
 * -------------------------
 * Scores completed trajectories by their accumulated action:
 *
 *   S[q] = ∫₀ᵀ L(q, q̇) dt = ∫₀ᵀ (T - V) dt
 *
 * Hamilton's Principle: the trajectory a physical system actually takes
 * is the one for which δS = 0. This is not an approximation — it is
 * exactly equivalent to Newton's laws.
 *
 * Natural motion minimises action. Trajectories are ranked by S —
 * the lowest action candidate is the most physically natural motion.
 *
 * @date 2026
 * @copyright Catherine Earl. All rights reserved.
 */

#include <Eigen/Dense>
#include <vector>
#include <array>
#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include "../utils/config.h"

#include "../robot_dynamics.h"

static constexpr int    N_DOF_L  = HNMG::DEGREE_FREEDOM;
static constexpr double DT_L     = HNMG::CONTROL_TIME_STEP;
static constexpr double ALPHA_L  = HNMG::ALPHA_BASE;

// ---------------------------------------------------------------------------
// Trajectory — a sequence of phase space points with timestamps
// ---------------------------------------------------------------------------
struct TrajectoryPoint {
    Eigen::Matrix<double, N_DOF_L, 1> q;   // configuration
    Eigen::Matrix<double, N_DOF_L, 1> p;   // momentum
    double t;                               // time
};

using Trajectory = std::vector<TrajectoryPoint>;

// ===========================================================================
// Part 3A — LiouvilleOperator
// ===========================================================================

/**
 * @brief Integrates phase space forward in time under the Liouville equation.
 *
 * The symplectic Euler integrator preserves the symplectic structure of phase
 * space — Liouville's theorem states that phase space volume is conserved
 * under Hamiltonian flow:
 *
 *   d/dt(∫∫ dq dp) = 0
 *
 * Standard (non-symplectic) integrators violate this, causing energy to drift
 * over time and producing physically inconsistent trajectories. The symplectic
 * integrator oscillates around the true energy but never drifts — valid always.
 *
 * Symplectic order: momentum updated first, then position using the NEW
 * momentum. This asymmetric update is what preserves the symplectic structure.
 */

class LiouvilleOperator {
    public:
    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------
    explicit LiouvilleOperator(const RobotDynamics& dynamics, unsigned seed = 42)
        : dyn_(dynamics), rng_(seed), normal_(0.0, 1.0) {}
    
    // -----------------------------------------------------------------------
    // Symplectic Euler step with stochastic diffusion
    //
    // Implements the Langevin equation:
    //   dq = (∂H/∂p) dt + √(2α) dWq
    //   dp = -(∂H/∂q) dt + √(2α) dWp
    //
    // Symplectic order — momentum first, then position using new momentum.
    // This is the key that preserves phase space volume.
    //
    // Standard Euler uses old momentum for both updates — energy drifts.
    // Symplectic Euler uses new momentum for position — energy conserved.
    //
    // @param q         current configuration (modified in place)
    // @param p         current momentum (modified in place)
    // @param alpha_t   diffusion coefficient at this timestep
    // @param dt        integration timestep
    // -----------------------------------------------------------------------
    

    private:
    const RobotDynamics& dyn_;
    std::mt19937 rng_;
    std::normal_distribution<double> normal_;
};
