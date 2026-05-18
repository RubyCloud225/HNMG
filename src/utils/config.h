#pragma once

#include <cstddef>

namespace HNMG {

// Configuration parameters for the humanoid motion generation system
// These parameters can be adjusted to tune the performance and behavior of the system
static constexpr int DEGREE_FREEDOM = 30;                // Number of degrees of freedom in the system
static constexpr double CONTROL_TIME_STEP = 0.001;      // Time step for control updates (s)
static constexpr double HBAR = 1.0545718e-34;           // Reduced Planck's constant in J*s
static constexpr double GRAVITY = 9.81;                 // Gravitational acceleration in m/s^2
static constexpr std::size_t N_PARTICLES = 1000;       // Number of particles for Monte Carlo sampling
static constexpr double ALPHA_BASE = 0.01; // Base diffusion co-efficient
static constexpr int N_SAMPLES = 10; // Trajectory
static constexpr int HORIZON = 100; // Planning horizon steps
static constexpr double GOAL_STIFFNESS = 1.0; // goal attraction 

} // namespace HNMG