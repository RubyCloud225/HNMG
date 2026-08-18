#pragma once
/**
 * @file natural_mode_evolution.h
 * @author Catherine Earl
 * @brief Part 4 — NaturalModeEvolution
 *
 * Decomposes the inertia matrix M(q) into its natural motion modes via
 * Jacobi eigendecomposition, propagates each mode independently, and
 * reconstructs the joint space trajectory.
 *
 * The inertia matrix M(q) defines a Riemannian metric on configuration space.
 * Its eigenvectors are the directions of natural motion — the directions in
 * which the robot moves most and least easily at a given configuration.
 *
 *   M(q) = V Λ Vᵀ
 *
 *   V = eigenvectors — natural motion directions (columns)
 *   Λ = diagonal matrix of eigenvalues — natural frequencies
 *
 * Small eigenvalue → light effective inertia → energetically cheap motion
 * Large eigenvalue → heavy effective inertia → energetically costly motion
 *
 * The natural mode propagator evolves the state along these modes:
 *
 *   q(t+dt) = V exp(-Λdt) Vᵀ q(t)
 *
 * Each mode evolves independently at its own natural frequency.
 * This is O(n) after the decomposition — fast for the inner planning loop.
 *
 * Adaptive diffusion:
 * The diffusion coefficient α_t is modulated by proximity to the goal
 * and local configuration uncertainty. Far from goal → diffuse more to
 * explore. Near goal → diffuse less to converge precisely.
 *
 * @date 2026
 * @copyright Catherine Earl. All rights reserved.
 */

