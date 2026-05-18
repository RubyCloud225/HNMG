# Humanoid & Robotic Arm: Complete Algorithm Reference (C++)

---

## 1. KINEMATICS

### Forward Kinematics (FK)
**What it does:** Given joint angles, compute end effector position and orientation.
**Algorithm:** Denavit-Hartenberg (DH) transformation matrices chained per joint.
```cpp
// DH Transform per joint
Eigen::Matrix4d dh_transform(double a, double d, double alpha, double theta) {
    Eigen::Matrix4d T;
    T << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta),
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
         0,           sin(alpha),             cos(alpha),            d,
         0,           0,                      0,                     1;
    return T;
}
// Chain: T_total = T1 * T2 * ... * Tn
```
**Key concept:** Rotation matrices (SO3) + translation = homogeneous transform (SE3).

---

### Inverse Kinematics (IK)
**What it does:** Given desired end effector pose, find joint angles.

**Analytical IK** — closed form, fast, robot-specific.

**Numerical IK — Jacobian Pseudoinverse:**
```cpp
// Iterative IK
Eigen::VectorXd ik_jacobian(Eigen::VectorXd q, Eigen::Vector3d target, int max_iter=100) {
    for (int i = 0; i < max_iter; i++) {
        Eigen::Vector3d pos = fk_position(q);
        Eigen::Vector3d error = target - pos;
        if (error.norm() < 1e-6) break;
        Eigen::MatrixXd J = compute_jacobian(q);         // 3xN
        Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
        q += J_pinv * error * 0.1;                       // step size 0.1
        q = clamp_joints(q);                             // respect joint limits
    }
    return q;
}
```
**Variants:** Damped Least Squares (DLS) for singularity robustness — adds λI to J*Jᵀ.

---

### Jacobian
**What it does:** Maps joint velocities to end effector velocities. J ∈ R^(6xN).
```
ẋ = J(q) * q̇
```
- **Geometric Jacobian** — cross products of joint axes
- **Analytical Jacobian** — partial derivatives of FK

**Singularities:** det(J*Jᵀ) → 0, robot loses DOF. Detect via manipulability measure: w = sqrt(det(J*Jᵀ))

---

## 2. DYNAMICS

### Newton-Euler Recursive Algorithm
**What it does:** Compute joint torques from desired accelerations (Inverse Dynamics).
**Forward pass:** propagate velocities and accelerations outward link by link.
**Backward pass:** propagate forces and torques inward.
```
τ = M(q)q̈ + C(q,q̇)q̇ + G(q)
```
- M(q) — inertia matrix
- C(q,q̇) — Coriolis/centrifugal matrix
- G(q) — gravity vector

---

### Lagrangian Dynamics
**What it does:** Derive equations of motion from energy.
```
L = T - V    (kinetic - potential energy)
d/dt(∂L/∂q̇) - ∂L/∂q = τ
```
More elegant than Newton-Euler, equivalent result. Your Hamiltonian mechanics background is directly applicable here — H = T + V is the Hamiltonian formulation of the same system.

---

### Forward Dynamics
Given torques, compute resulting accelerations:
```
q̈ = M(q)⁻¹ * (τ - C(q,q̇)q̇ - G(q))
```
Used in simulation and model predictive control.

---

## 3. TRAJECTORY PLANNING

### Joint Space Trajectories
**Cubic polynomial:** smooth position and velocity, zero acceleration at endpoints.
```cpp
// Cubic spline between q0 and qf in time T
double cubic_traj(double q0, double qf, double v0, double vf, double t, double T) {
    double a0 = q0, a1 = v0;
    double a2 = (3*(qf-q0) - T*(2*v0+vf)) / (T*T);
    double a3 = (-2*(qf-q0) + T*(v0+vf)) / (T*T*T);
    return a0 + a1*t + a2*t*t + a3*t*t*t;
}
```
**Quintic polynomial:** also constrains acceleration — smoother, better for torque.
**Minimum jerk trajectory:** minimises rate of change of acceleration — most natural human-like motion.

---

### Cartesian Space Trajectories
**Linear interpolation** in position + SLERP for orientation:
```cpp
// SLERP between quaternions
Eigen::Quaterniond slerp(Eigen::Quaterniond q0, Eigen::Quaterniond q1, double t) {
    return q0.slerp(t, q1);
}
```
**Why SLERP not LERP for rotation:** quaternion space is non-Euclidean, linear interpolation produces non-constant angular velocity and non-unit quaternions.

---

### Collision Aware Planning

**RRT (Rapidly-Exploring Random Trees):**
```
1. Sample random config q_rand
2. Find nearest node q_near in tree
3. Extend toward q_rand by step size
4. Check collision
5. Add if collision free
6. Repeat until goal reached
```
**RRT\*:** adds rewiring step for asymptotically optimal paths.

**A\* in configuration space:** grid based, optimal but memory intensive for high DOF.

**CHOMP / STOMP:** gradient based trajectory optimisation minimising collision cost and smoothness cost simultaneously.

---

## 4. CONTROL

### PID Control
```cpp
class PID {
    double kp, ki, kd, integral=0, prev_error=0;
public:
    PID(double p, double i, double d) : kp(p), ki(i), kd(d) {}
    double compute(double error, double dt) {
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp*error + ki*integral + kd*derivative;
    }
};
```
**Limitation:** linear, doesn't account for robot dynamics. Fine for slow motions, insufficient for fast dynamic tasks.

---

### Computed Torque Control (Model Based)
```
τ = M(q)(q̈_d + Kd*ė + Kp*e) + C(q,q̇)q̇ + G(q)
```
Cancels nonlinear dynamics, reduces to linear error dynamics. Requires accurate dynamic model — model errors become disturbances.

---

### Model Predictive Control (MPC)
**What it does:** Optimise control over a receding horizon.
```
min Σ ||x_k - x_ref||²_Q + ||u_k||²_R
subject to: x_{k+1} = f(x_k, u_k)
            x_min ≤ x ≤ x_max
            u_min ≤ u ≤ u_max
```
- Horizon N steps ahead
- Solve QP at each timestep, apply first control input
- Naturally handles constraints — joint limits, torque limits, collision avoidance

**Your Hamiltonian thinking applies here:** the optimal control Hamiltonian H = p·f(x,u) + L(x,u) is the foundation of the Pontryagin Maximum Principle which MPC approximates iteratively.

---

### Impedance Control
**What it does:** Control the mechanical impedance — make the robot feel like a spring-damper to external forces. Critical for contact tasks and human interaction.
```
F = M_d * (ẍ - ẍ_d) + B_d * (ẋ - ẋ_d) + K_d * (x - x_d)
```
- M_d, B_d, K_d — desired inertia, damping, stiffness
- Compliant in some directions, stiff in others

**Force control:** regulate contact force directly using F/T sensor feedback.

---

### Whole Body Control (WBC)
For humanoids — coordinate all joints simultaneously to achieve multiple tasks with priorities:
```
Priority 1: Don't fall (balance)
Priority 2: Achieve end effector goal
Priority 3: Avoid joint limits
Priority 4: Minimise energy
```
Implemented via hierarchical QP — each priority level solved in null space of higher priority tasks.

---

## 5. STATE ESTIMATION & PERCEPTION

### Extended Kalman Filter (EKF)
**What it does:** Estimate robot state (position, velocity, orientation) from noisy sensor data.
```
Predict:
x̂_k|k-1 = f(x̂_k-1, u_k)
P_k|k-1 = F_k * P_k-1 * F_kᵀ + Q_k

Update:
K_k = P_k|k-1 * H_kᵀ * (H_k * P_k|k-1 * H_kᵀ + R_k)⁻¹
x̂_k = x̂_k|k-1 + K_k * (z_k - h(x̂_k|k-1))
P_k = (I - K_k * H_k) * P_k|k-1
```
- F_k — Jacobian of process model
- H_k — Jacobian of observation model
- Q — process noise covariance
- R — measurement noise covariance
- K — Kalman gain (how much to trust measurement vs prediction)

**Your epsilon notation:** ε tolerance appears in convergence criteria for filter initialisation.

---

### IMU Integration
**What it does:** Track orientation and velocity from accelerometer + gyroscope.
```cpp
// Gyroscope integration for orientation
Eigen::Quaterniond integrate_gyro(Eigen::Quaterniond q, Eigen::Vector3d omega, double dt) {
    Eigen::Quaterniond omega_q(0, omega.x()*0.5, omega.y()*0.5, omega.z()*0.5);
    return Eigen::Quaterniond(q.coeffs() + (q * omega_q).coeffs() * dt).normalized();
}
```
**Drift:** gyroscope integrates error over time. Correct with accelerometer (gravity reference) and magnetometer (heading). EKF fuses all three.

---

### SLAM (Simultaneous Localisation and Mapping)
**What it does:** Build map of environment while tracking robot position within it.

**Graph SLAM:** nodes are poses, edges are relative measurements. Optimise graph:
```
min Σ ||z_ij - h(x_i, x_j)||²_Ω
```
**Factor Graph:** more general formulation. Libraries: GTSAM, g2o.

**Visual SLAM (vSLAM):** ORB-SLAM3, OpenVSLAM — feature extraction, matching, pose estimation, loop closure.

**LiDAR SLAM:** ICP (Iterative Closest Point) for scan matching:
```
min Σ ||p_i - R*q_i - t||²
```

---

### Point Cloud Processing
```cpp
// Voxel downsampling — reduce point cloud density
// Normal estimation — surface normals for grasping
// ICP — align two point clouds
// RANSAC — robust plane/object fitting
```

---

## 6. GRASPING & MANIPULATION

### Grasp Planning
**Force closure:** grasp is stable if contact forces can resist arbitrary external wrenches.
**Quality metrics:** smallest singular value of grasp matrix G — higher = more stable.

**Grasp matrix:**
```
G * f_c = w_ext
```
G maps contact forces to object wrench. Compute via contact normals and moment arms.

### Grasp Pose Generation
- **Geometric:** sample antipodal point pairs on object surface
- **Learning based:** GraspNet, Contact-GraspNet — predict grasp poses from point cloud
- **GPD (Grasp Pose Detection):** CNN on point cloud local patches

---

## 7. BALANCE & LOCOMOTION (HUMANOID)

### Zero Moment Point (ZMP)
**What it does:** Point on ground where net ground reaction moment is zero. If ZMP stays within support polygon robot doesn't fall.
```
ZMP_x = (Σ m_i * (ẍ_i + g) * x_i) / (Σ m_i * (z̈_i + g))
```

### Centroidal Dynamics
```
f = m * ẍ_com        (linear momentum rate)
τ = İ_com * ω + I_com * ω̇    (angular momentum rate)
```
Control centre of mass trajectory to maintain balance.

### Linear Inverted Pendulum Model (LIPM)
Simplification for walking — treats robot as point mass on massless leg:
```
ẍ = (g/z_com) * (x - x_zmp)
```
Analytically tractable, used for footstep planning.

### Footstep Planning
```
1. Plan ZMP trajectory
2. Compute COM trajectory from LIPM
3. Generate swing foot trajectory (cubic spline)
4. Use WBC to track all simultaneously
```

---

## 8. SENSING

### Force/Torque Sensing
6-axis F/T sensors at wrist. Used for:
- Contact detection
- Impedance control
- Grasp quality estimation
- Collision detection (safety)

### Joint Encoders
Position → velocity via finite difference (noisy) or Kalman filter.
Torque estimation from motor current × torque constant.

### Vision Pipeline
```
Camera → Undistortion → Feature Detection → 
Object Recognition → Pose Estimation → Grasp Planning
```

**Camera calibration:** Zhang's method. Intrinsic matrix K, distortion coefficients.
**Pose estimation:** PnP (Perspective-n-Point) from 2D-3D correspondences.

---

## 9. SAFETY & REAL TIME

### Real Time Requirements
- Control loop: 1kHz minimum for torque control
- Perception: 30-100Hz
- Planning: 1-10Hz (slower, can be replanned)

**C++ real time:** avoid dynamic allocation in control loop, use fixed size buffers, lock free data structures for inter-thread communication, POSIX real time scheduling (SCHED_FIFO).

```cpp
// Set real time priority
struct sched_param param;
param.sched_priority = 99;
pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
```

### Safety Monitors
```cpp
// Joint limit checking
bool check_joint_limits(const Eigen::VectorXd& q, const Eigen::VectorXd& q_min, const Eigen::VectorXd& q_max) {
    return (q.array() >= q_min.array()).all() && (q.array() <= q_max.array()).all();
}

// Torque limit
bool check_torque_limits(const Eigen::VectorXd& tau, double tau_max) {
    return tau.array().abs().maxCoeff() <= tau_max;
}

// Velocity limit
bool check_velocity_limits(const Eigen::VectorXd& dq, double dq_max) {
    return dq.array().abs().maxCoeff() <= dq_max;
}
```

### Collision Detection
**Bounding volume hierarchies (BVH):** OBB trees for fast collision queries.
**FCL (Flexible Collision Library):** standard C++ collision detection.
**Signed Distance Fields (SDF):** precomputed distance to nearest obstacle, fast gradient for repulsive potential fields.

---

## 10. KEY C++ LIBRARIES

| Library | Purpose |
|---|---|
| Eigen | Linear algebra, matrices, quaternions |
| Pinocchio | Rigid body dynamics, URDF parsing |
| FCL | Collision detection |
| OMPL | Motion planning algorithms |
| OpenCV | Vision, camera calibration |
| PCL | Point cloud processing |
| ROS2 | Middleware, sensor integration |
| OSQP | QP solver for MPC/WBC |
| GTSAM | Factor graph SLAM |

---

## 11. YOUR TRANSFERABLE STRENGTHS — FRAME THESE

| Your Work | Robotics Equivalent |
|---|---|
| Hamiltonian evolution | Lagrangian/Hamiltonian dynamics, MPC optimal control |
| Phase space (position + momentum) | State space (q, q̇) — identical formalism |
| Symplectic Euler integrator | Standard integrator for Hamiltonian systems in dynamics |
| KL divergence minimisation | Cost function minimisation in trajectory optimisation |
| Conjugate gradient solver | Used in WBC and MPC QP solving |
| SINDy streaming compression | Continuous sensor data dimensionality reduction |
| Zero dependency C++ | Real time control loop implementation |
| ARM SIMD optimisation | Embedded motor controller targets same architecture |
| Taylor expansion propagator | Standard numerical integration in dynamics |
| Mirror Ancilla Hilbert space | Null space methods in WBC |

---

## 12. INTERVIEW FRAMING

**Lead with:** state estimation, continuous data pipelines, low latency C++, real time systems.

**Then show depth with:** Hamiltonian mechanics for dynamics, phase space for state representation, conjugate gradient for QP solving.

**Don't open with:** quantum, Clifford algebra, Hamiltonian encoder. Let them ask.

**The one sentence:** "I work at the intersection of physics-based modelling and hardware-level C++ optimisation — which maps directly onto dynamics modelling and real time control."

---

*Catherine Earl — May 2026*