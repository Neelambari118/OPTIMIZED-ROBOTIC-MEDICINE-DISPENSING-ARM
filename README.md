<img width="830" height="643" alt="image" src="https://github.com/user-attachments/assets/4874a96a-2c04-4130-95b8-3d65033d32e6" /># Optimized Robotic Medicine-Dispensing Arm

![MATLAB](https://img.shields.io/badge/MATLAB-R2023a-orange.svg)
![License](https://img.shields.io/badge/License-MIT-blue.svg)
![Status](https://img.shields.io/badge/Status-Research_Prototype-green.svg)

##  Abstract
This project implements a robust control framework for a **4-DOF SCARA manipulator** designed for automated pharmaceutical dispensing. Unlike conventional geometric solvers, this system formulates Inverse Kinematics (IK) as a **constrained numerical optimization problem** using Jacobian-based gradient descent.

Integrated with a physics-engine based on **Euler-Lagrange dynamics**, the controller performs real-time gravity compensation and torque estimation, ensuring sub-millimeter accuracy ($< 10^{-5}$ m) and mechanical stability under variable payloads.

##  Mathematical Framework
The core control logic relies on two primary mathematical formulations derived in the project report.

### 1. Optimization-Based Inverse Kinematics
Instead of analytical solutions, we minimize the squared Euclidean error $E(q)$ between the target and current end-effector positions:
$$E(q) = || P_{target} - P_{current}(q) ||^2$$

The joint update rule utilizes the **Jacobian Transpose** method for singularity avoidance:
$$q_{new} = q_{old} + \alpha J^T (P_{target} - P_{current})$$

### 2. Physics-Aware Dynamics
To ensure realistic motion, motor torques are computed using the **Euler-Lagrange equation**:
$$L = T - V$$
$$\tau = M(q)\ddot{q} + C(q, \dot{q})\dot{q} + G(q)$$

Where $G(q)$ handles dynamic **gravity compensation**, automatically adjusting vertical force ($F_z$) upon detection of the 0.2 kg payload mass.

##  Key Features
* **Singularity Robustness:** Numerical optimization prevents high-velocity spikes near workspace boundaries.
* **Jerk-Free Motion:** Quintic polynomial interpolation generates smooth S-curve velocity profiles, preventing liquid spillage.
* **Dynamic Payload Detection:** System detects mass changes ($t \approx 90$) and compensates vertical torque instantaneously.
* **3D Visualization:** Full kinematic simulation including workspace validation and trajectory tracing.

##  Repository Structure
```text
├── src/
│   └── main.m               # Monolithic simulation script
│                            # (Contains: IK Solver, Euler-Lagrange Dynamics, Trajectory Planner)
├── docs/
│   └── REPORT.pdf           # Full Technical Report
├── .gitignore
└── README.md


