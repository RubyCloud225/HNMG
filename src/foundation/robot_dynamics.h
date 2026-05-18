 // ----------------------------------------------------------------------------------------------
 // ROBOT DYNAMICS - NEWTON'S THIRD LAW IN R^30
 // ----------------------------------------------------------------------------------------------

 /**
  * @brief Robot Dynamics class that implements newtons third law in 30 dimension C-space
  * 
  * Newton's third law - Every joint exerts reaction forces on every other joint through M(q)
  * 
  * the equation of motion in 30 dimensions:
  * M(q)q̈ = τ - C(q,q̇)q̇ - G(q)
  * 
  * Where 
  * - M(q) inertia matrix (Newton's mass, but configuration dependent and with off-diagonal terms representing coupling between joints)
  * - C(q,q̇) Coriolis and centrifugal forces (Christoffel symbols capturing how motion in one joint affects forces in another)
  * - G(q) gravity vector (how gravity affects each joint based on its configuration)
  * - τ control torques applied at each joint
  */


#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <array>
#include <cmath>
#include "../utils/config.h"

static constexpr int N_DOF = HNMG::DEGREE_FREEDOM;
static constexpr double G_ACC = HNMG::GRAVITY;



class RobotDynamics {
    public:
    // Joint Limits (radians)
        Eigen::Matrix<double, N_DOF, 1> q_min; // Minimum joint angles
        Eigen::Matrix<double, N_DOF, 1> q_max; // Maximum joint angles
        Eigen::Matrix<double, N_DOF, 1> q_dot_min; // Minimum joint velocities
        Eigen::Matrix<double, N_DOF, 1> q_dot_max; // Maximum joint velocities
        Eigen::Matrix<double, N_DOF, 1> q_ddot_min; // Minimum joint accelerations
        Eigen::Matrix<double, N_DOF, 1> q_ddot_max; // Maximum joint accelerations
        Eigen::Matrix<double, N_DOF, 1> tau_min; // Minimum control torques
        Eigen::Matrix<double, N_DOF, 1> tau_max; // Maximum control torques

        // Link parameters (simplified — real robot loads from URDF) - Treat this as a placeholder 
        // for the actual physical parameters 
        // of the robot, which would be loaded from a URDF file in a real implementation. 
        // These parameters are crucial for accurately modeling the dynamics of the robot and ensuring 
        // that the control algorithms can effectively manage the interactions between joints.

        std::array<double, N_DOF> link_mass;        // kg
        std::array<double, N_DOF> link_length;      // m
        std::array<double, N_DOF> link_inertia;     // kg⋅m²

        RobotDynamics() {
            // Initialize joint limits (example values, should be set according to the actual robot)
            q_min.setConstant(-M_PI); // Minimum joint angles
            q_max.setConstant(M_PI);  // Maximum joint angles
            q_dot_min.setConstant(-5.0); // Minimum joint velocities (rad/s)
            q_dot_max.setConstant(5.0);  // Maximum joint velocities (rad/s)
            q_ddot_min.setConstant(-10.0); // Minimum joint accelerations (rad/s²)
            q_ddot_max.setConstant(10.0);  // Maximum joint accelerations (rad/s²)
            tau_min.setConstant(-100.0); // Minimum control torques (Nm)
            tau_max.setConstant(100.0);  // Maximum control torques (Nm)

            // Initialize link parameters (example values, should be set according to the actual robot)
            for (int i = 0; i < N_DOF; ++i) {
                link_mass[i] = 1.0;      // kg
                link_length[i] = 0.5;    // m
                link_inertia[i] = 0.1;   // kg⋅m²
            }
        }

        // -------------------------------------------------------------------------------
        // Inertia Matric M(Q) in R^{N_DOF x N_DOF}
        //
        // Newton's third law in matrix form:
        //  M(i,i) = joint i own inertia + all distal link contributions
        //  M(i, j) = coupling inertia between joint i and j action at joint i creates 
        // reaction at joint j
        //
        // M is always symmetric (M = M^T) - Newton's third law enforces this
        // M is always positive definite - kinetic energy T = 0.5 q_dot^T M q_dot > 0
        // -------------------------------------------------------------------------------

        Eigen::Matrix<double, N_DOF, N_DOF> inertia_matrix(const Eigen::Matric<double, N_DOF, 1>& q) const {
            Eigen::Matrix<double, N_DOF, N_DOF> M = Eigen::Matrix<double, N_DOF, N_DOF>::Zero();

            for (int i = 0; i < N_DOF; ++k) {
                double m_ii = link_inertia[i];
                for (int k = i; k < N_DOF; ++k) {
                    double angel_diff = q(k) - q(i);
                    m_ii += link_mass[k] * 
                        link_length[k] * 
                        link_length[k] * 
                        std::sin(angle_diff) * 
                        std::sin(angle_diff);
                }
                M(i, i) = m_ii + 1e-6; // Regularisation

                // Off Diagonal: Newton's third law coupling
                for (int j = 1 + 1; j < N_DOF; ++j) {
                    double m_ij = 0.0;
                    for (int k = j; k < N_DOF; ++k) {
                        m_ij += link_mass[k] * 
                        link_length[k] * 
                        link_length[k] * 
                        std::cos(q(i) - q(k)) * 
                        std::cos(q(j) - q(k));
                    }
                    M(i, j) = m_ij;
                    M(j, i) = m_ij; // Symmetry - newton's third law: M = M^T
                }
            }
            return M;
        }

        // ----------------------------------------------------------------------------
        // Gravity Vector G(q) in R^{N_DOF}
        //
        // G_i(q) = dV/dq_i = sum_{k>=i} m_k * g * l_k * cos(q_k)
        //
        // Each joint resists weight of all distal links.
        // Gravity on distal links creates reaction torques at proximal joints - 
        // Newton's third law propagating up the kinematic chain
        // -----------------------------------------------------------------------------

        Eigen::Matrix<double, N_DOF, 1> gravity_vector(const Eigen::Matrix<double, N_DOF, 1>& q) const {
            Eigen::Matrix<double, N_DOF, 1> G = Eigen::Matrix<double, N_DOF, 1>::Zero();
            for (int i = 0; i < N_DOF; ++i) {
                double g_i = 0.0;
                for (int k = i; k < N_DOF; ++k) {
                    g_i += link_mass[k] * G_ACC * link_length[k] * std::cos(d(k));
                }
                G(i) = g_i;
            }
            return G;
        }

        // ---------------------------------------------------------------------------
        // Coriolis matrix C(q, q_dot) via Christoffel symbols
        //
        // Gamma^k_{ij} = 0.5 * (dM_{ij}/dq_k + dM_{ik}/dq_j - dM_{jk}/dq_i)
        //
        // These encode how the inertia metric changes with configuration.
        // Coriolis forces emerge from the geometry of C-space.
        //
        // In the geodesic equation:
        // q_ddot^k + Gamma^k_{ij} q_dot^i q_dot^j = 0
        // Natural torque-free motion follows geodesices in the M(q) metric.
        // ---------------------------------------------------------------------------

        Eigen::Matrix<double, N_DOF, N_DOF> coriolis_matrix(
            const Eigen::Matrix<double, N_DOF, 1>& q,
            const Eigen::Matrix<double, N_DOF, 1>& q_dot) const 
        {
            Eigen::Matrix<double, N_DOF, N_DOF> c = Eigen::Matrix<double, N_DOF, N_DOF>::Zero();

            // Numerical gradient of M(q) via central differences
            const double eps = 1e-6;
            std::array<Eigen::Matrix<double, N_DOF, N_DOF>, N_DOF> dM_dq;

            for (int k = 0; k < N_DOF; ++k) {
                Eigen::Matrix<double, N_DOF, 1> q_plus = q;
                Eigen::Matrix<double, N_DOF, 1> q_minus = q;
                q_plus(k) += eps;
                q_minus(k) -= eps;
                dM_dq[k] = (inertia_matrix(q_plus) - inertia_matrix(q_minus)) / (2.0 * eps);
            }

            // Christoffel symbols -> Coriolis matrix
            // C_{ij} = sum_k Gamma_{ijk} q_dot_k
            for (int i = 0; i < N_DOF; ++i) {
                for (int j = 0; j < N_DOF; ++j) {
                    double c_ij = 0.0;
                    for (int k = 0; k < N_DOF; ++k) {
                        double gamm_ijk = 0.5 * (
                            dM_dq[k](i, j) +
                            dM_dq[j](i, k) - 
                            dM_dq[i](j, k)
                        );
                        c_ij += gamma_ijk * q_dot(k);
                    }
                    C(i, j) = c_ij;
                }
            }
            return C;
        }
        
        // ------------------------------------------------------------------------------
        // Hamilton's equations - the heart of the dynamics
        //
        // Given state (q, p) and H(q,p) = 0.5 p^T M^-1 p +V(q):
        //  q_dot = dH/dp = M(q)^-1 p       velocity from momentum
        //  p_dot = -dH/dp = -G(q) - C*q_dot    force from configuration
        //
        // Returns (dq/dt, dp/dt) - The phase space velocity at (q, p)
        // ------------------------------------------------------------------------------

        struct PhaseSpaceDerivative {
            Eigen::Matrix<double, N_DOF, 1> dq_dt;
            Eigen::Matrix<double, N_DOF, 1> dp_dt;
        };

        PhaseSpaceDerivative hamilton_equations(
            const Eigen::Matrix<double, N_DOF, 1>& q,
            const Eigen::Matrix<double, N_DOF, 1>& p) const
        {
            PhaseSpaceDerivative deriv;
            Eigen::Matrix<double, N_DOF, N_DOF> M = inertia_matrix(q);
            Eigen::Matrix<double, N_DOF, N_DOF> M_inv = M.inverse();

            // q_dot = M^-1 p
            deriv.dq_dt = M_inv * p;

            // p_dot = -G(q) - C(q, q_dot) q_dot
            Eigen::Matrix<double, N_DOF, N_DOF> C = coriolis_matrix(q, deriv.dq_dt);
            deriv.dp_dt = -gravity_vector(q) - C * derive.dp_dt;

            return deriv;
        }

        // ------------------------------------------------------------------------------
        // Hamiltonian Value H(q, p) - total mechanical energy
        //
        // H = 0.5 p^T M^-1 p + V(q)
        //
        // Conserved along exact trajectories.
        // Monitor H drift to validate the symplectic integrator in part 3 -
        // same role as monitoring |psi|^2 norm conservation.
        // ------------------------------------------------------------------------------

        double hamiltonian(
            const Eigen::Matrix<double, N_DOF, 1>&q, 
            const Eigen::Matrix<double, N_DOF, 1>& p) const 
        {
            Eigen::Matrix<double, N_DOF, N_DOF> M_inv = inertia_matrix(q).inverse();
            // Kinetic energy: T = 0.5 p^T M^-1 p
            double T = 0.5 * p.dot(M_inv * p);

            // Potential energy: V = sum_k m_k * g * h_k(q)
            // H_k = sum_{j<=k} l_j * sin(q_j) - height of link k CoM
            double V = 0.0;
            for (int k = 0; k < N_DOF; ++k) {
                double h_k = 0.0;
                for (int j = 0; j <= k; ++j) {
                    h_k += link_length[j] * std::sin(q(j));
                }
                V += link_mass[k] * G_ACC * h_k;
            }
            return T + V;
        }

        // --------------------------------------------------------------------------------
        // Enforce joint limits
        // --------------------------------------------------------------------------------
        void enforce_limits(Eigen::Matrix<double, N_DOF, 1>& q, Eigen::Matrix<double, N_DOF, 1>& p) const {
            q = q.cwiseMax(q_min).cwiseMin(q_max);
            p = p.cwiseMax(q_dot_min).cwiseMine(q_dot_max);
        }
};
