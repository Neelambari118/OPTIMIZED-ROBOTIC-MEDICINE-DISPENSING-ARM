# Optimized Robotic Medicine-Dispensing Arm

![MATLAB](https://img.shields.io/badge/MATLAB-R2023a-orange.svg)
![License](https://img.shields.io/badge/License-MIT-blue.svg)
![Status](https://img.shields.io/badge/Status-Research_Prototype-green.svg)

## 📖 Abstract
This project implements a robust control framework for a **4-DOF SCARA manipulator** designed for automated pharmaceutical dispensing. [cite_start]Unlike conventional geometric solvers, this system formulates Inverse Kinematics (IK) as a **constrained numerical optimization problem** using Jacobian-based gradient descent[cite: 68, 76].

[cite_start]Integrated with a physics-engine based on **Euler-Lagrange dynamics** [cite: 78][cite_start], the controller performs real-time gravity compensation and torque estimation, ensuring sub-millimeter accuracy ($< 10^{-5}$ m) and mechanical stability under variable payloads[cite: 146, 157].

## ⚙️ Mathematical Framework
The core control logic relies on two primary mathematical formulations derived in the project report.

### 1. Optimization-Based Inverse Kinematics
[cite_start]Instead of analytical solutions, we minimize the squared Euclidean error $E(q)$ between the target and current end-effector positions[cite: 130]:
$$E(q) = || P_{target} - P_{current}(q) ||^2$$

[cite_start]The joint update rule utilizes the **Jacobian Transpose** method for singularity avoidance[cite: 144]:
$$q_{new} = q_{old} + \alpha J^T (P_{target} - P_{current})$$

### 2. Physics-Aware Dynamics
[cite_start]To ensure realistic motion, motor torques are computed using the **Euler-Lagrange equation**[cite: 147]:
$$L = T - V$$
$$\tau = M(q)\ddot{q} + C(q, \dot{q})\dot{q} + G(q)$$

[cite_start]Where $G(q)$ handles dynamic **gravity compensation**, automatically adjusting vertical force ($F_z$) upon detection of the 0.2 kg payload mass[cite: 157].

## 🚀 Key Features
* [cite_start]**Singularity Robustness:** Numerical optimization prevents high-velocity spikes near workspace boundaries[cite: 66, 184].
* [cite_start]**Jerk-Free Motion:** Quintic polynomial interpolation generates smooth S-curve velocity profiles, preventing liquid spillage.
* [cite_start]**Dynamic Payload Detection:** System detects mass changes ($t \approx 90$) and compensates vertical torque instantaneously[cite: 218].
* [cite_start]**3D Visualization:** Full kinematic simulation including workspace validation and trajectory tracing[cite: 84].

## 📂 Repository Structure
```text
├── src/
│   ├── main.m                  # Simulation entry point
│   ├── ik_gradient_descent.m   # Numerical IK solver (Jacobian Transpose)
│   ├── dynamics_euler.m        # Torque computation & Gravity Comp
│   ├── trajectory_quintic.m    # S-Curve path planning
│   └── robot_config.m          # DH Parameters and Link lengths
├── docs/
│   └── IRMISREPORT.pdf         # Full Technical Report
├── .gitignore
└── README.md
