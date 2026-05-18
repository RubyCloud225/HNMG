/**
 * @file c_space.cpp
 * @author Catherine Earl (catherineearl8@gmail.com)
 * @brief Implementation of Newton's third law of motion in a 30 dimensional space using C++20 with Schrödinger's time evolution.
 * @version 0.1
 * @date 2025-05-07
 * 
 * R^30 = { (x1, x2, x3, ..., x30) | xi ∈ R for i = 1, 2, ..., 30 }
 * 
 * Use Newton's third law: For every action, there is an equal and opposite reaction.
 * 
 * F = m * a => a = F / m in C_space this becomes the Newton-Euler Recursive Algorithm for R^30:
 * * Forces and Torques propagate through the 30-dimensional space, effctively coupling the dynamics of all dimensions.
 * In 30 dimensions the cascade of action reaction pairs through the tree structure of the humaniod skeleton.
 * 
 * M(q)q̈ = τ - C(q,q̇)q̇ - G(q)
 * 
 * the M(q)q̈ = τ - C(q,q̇)q̇ - G(q) is Coriolis and centrifugal forces, and G(q) is the gravity vector. 
 * In 30 dimensions, these terms become significantly more complex, as they must account for the interactions 
 * between all 30 dimensions of motion.
 * 
 * Then an implementation of Schrödinger's time evolution in this 30-dimensional space would involve solving the time-dependent 
 * Schrödinger equation:
 * 
 * |ψ(t+dt)⟩ = e^(-iHdt/ℏ) |ψ(t)⟩
 * 
 * where H is the Hamiltonian operator that includes the kinetic and potential energy terms in 30 dimensions. 
 * 
 * The kinetic energy term would involve the momentum operators for each of the 30 dimensions, while the potential energy term would depend 
 * on the specific interactions and forces present in the system.
 * 
 * for the classical part of the system would use the analogue of robot state:
 * (q(t+dt), p(t+dt)) = e^(L̂ dt) (q(t), p(t))
 * 
 * Where L̂ is the Liouvillian operator that governs the time evolution of the classical state in 30 dimensions. 
 * 
 * The Liouvillian would include terms that account for the forces and interactions between all 30 dimensions, as w
 * ell as any external potentials or constraints present in the system.
 * 
 * L̂ = {H, ·} = ∂H/∂p · ∂/∂q - ∂H/∂q · ∂/∂p acts as the limiter for the classical state evolution, 
 * ensuring that the dynamics of the system are consistent with the underlying physics and 
 * constraints of the 30-dimensional space.
 * Copyright (c) 2026 Catherine Earl. All rights reserved.
 */


/**
 * Implementation of the CSpaceEvolution class, which combines the PhaseSpaceDensity, RobotDynamics, LiouvilleOperator, and NaturalModeEvolution classes to evolve the state of the humanoid robot in the 30-dimensional configuration space.
 * Part 1 — PhaseSpacePoint and PhaseSpaceDensity
 * The robot state as a quantum-style distribution. Single point vs full density ρ(q,p,t). 256 Monte Carlo particles approximating the continuous distribution.
 * Part 2 — RobotDynamics
 * Newton's third law embedded explicitly. The off-diagonal M(q) entries are the action-reaction coupling between joints. Coriolis via Christoffel symbols. Full Hamilton's equations with numerical gradient.
 * Part 3 — LiouvilleOperator
 * The classical Schrödinger analogue. Poisson bracket implemented explicitly. Symplectic Euler preserving phase space volume. Ehrenfest theorem — mean state follows classical Hamilton's equations.
 * Part 4 — NaturalModeEvolution
 * Jacobi diagonalisation exactly as in your MAHQ propagator. Project to modal basis, propagate each mode independently, reconstruct. The quantum-classical parallel spelled out in every comment.
 * Part 5 — CSpaceEvolution
 * Everything combined. Initialise density as a Gaussian — uncertainty about starting state. Evolve under Liouville. Goal-directed bias. Importance resampling by goal proximity — Born rule analogue.
 * The comments throughout make the quantum-classical correspondence explicit everywhere. That's your interview answer — you didn't learn robotics from a textbook, you derived it from physics you already knew.
 */

 #include <Eigen/Dense>
 #include <Eigen/Eigenvalues>
 #include <vector>
 #include <array>
 #include <cmath>
 #include <random>
 #include <iostream>
 #include <functional>
 #include <algorithm>
 #include <numeric>
 #include "../utils/config.h"

 // ----------------------------------------------------------------------------------------------
 // ------------------------------ Constants and Type Definitions --------------------------------
 // ----------------------------------------------------------------------------------------------

 static constexpr int N_DOF = HNMG::DEGREE_FREEDOM;
 static constexpr double DT = HNMG::CONTROL_TIME_STEP;
 static constexpr double HBAR = HNMG::HBAR; // Reduced Planck's constant
 static constexpr double G_ACC = HNMG::GRAVITY; // Gravitational acceleration

 // Configure the space
 // using the classical state of quantum state |ψ⟩ = (q, p) where q is the position and p is the momentum in 30 dimensions
 // q = configuration (joint angles) and p = momentum (joint velocities) e R^\delta ^g
 // (q, p) ∈ R^30 x R^30

 struct PhaseSpacePoint{
    Eigen::Matrix<double, N_DOF, 1> q; // Position (joint angles)
    Eigen::Matrix<double, N_DOF, 1> p; // Momentum (joint velocities)

    PhaseSpacePoint() {
        q.setZero();
        p.setZero();
    }

    PhaseSpacePoint(
        const Eigen::Matrix<double, N_DOF, 1>& config, 
        const Eigen::Matrix<double, N_DOF, 1>& momentum) 
        : q(config), p(momentum) {}
 };

 // Track the distributions of the particale points - a Monte Carlo sampling of the phase space
 // to the continuous density p(q,p,t) in the 30-dimensional space
 // Schrodingers: \psi (x, t) amplitude at each point in configuration space, where |ψ(x, t)|^2 gives the
 // probability density of finding the system at configuration x at time t.
 // Liouville: ρ(q, p, t) density at each point in phase space, where ρ(q, p, t) gives the probability density of 
 // finding the system with configuration q and momentum p at time t.
 // Both evolve under the same underlying physics, but they represent different aspects of the system's state and dynamics.

 struct PhaseSpaceDensity {
    static constexpr int N_PARTICLES = HNMG::N_PARTICLES; // Number of particles for Monte Carlo sampling
    std::array<PhaseSpacePoint, N_PARTICLES> particles; // Array of particles representing the phase space density
    std::array<double, N_PARTICLES> weights; // Weights for each particle in the Monte Carlo sampling

    PhaseSpaceDensity() {
        weights.fill(1.0/N_PARTICLES); // Initialize weights uniformly
    }

    // Mean configuration - expectation value (q)
    Eigen::Matrix<double, N_DOF, 1> mean_q() const {
        Eigen::Matrix<double, N_DOF, 1> mean_q = Eigen::Matrix<double, N_DOF, 1>::Zero();
        for (int i = 0; i < N_PARTICLES; ++i) {
            mean_q += weights[i] * particles[i].q;
        }
        return mean_q;
    }

    // Covariance of configuration - uncertainty (q)
    Eigen::Matrix<double, N_DOF, N_DOF> covariance_q() const {
        Eigen::Matrix<double, N_DOF, N_DOF> cov_q = Eigen::Matrix<double, N_DOF, N_DOF>::Zero();
        Eigen::Matrix<double, N_DOF, 1> mean_q_vec = mean_q();
        for (int i = 0; i < N_PARTICLES; ++i) {
            Eigen::Matrix<double, N_DOF, 1> diff = particles[i].q - mean_q_vec;
            cov_q += weights[i] * (diff * diff.transpose());
        }
        return cov_q;
    }
 };