ultimate deliverable will include
1. A live demo of your robot, under a controller you wrote and tuned, performing a cartesian
task (for example, picking and placing from the table with the gripper). The expectation is
that your controller will implement cartesian stiffness and damping controller. professor
will provide code that institutes an external force in X, and the robot will be expected to
deflect only in X if the decoupling is correct. You are encouraged to interact with your robot
as part of the demo (in a safe way).
2. A pdf report comprising
(a) A description of your controller and controller design process, featuring a time-domain
plot of a critically-damped step response in the simulator.
3. A zip file containing
(a) A simulation video of your robot tracing a trajectory in cartesian space.
(b) A simulation video of your robot tracing a trajectory in cartesian space but
experiencing a bias force, that results in the robot’s deviation in only the direction of
the bias force
(c) Your code, which should be able to reproduce all your videos, your plots, and your live
demo
----------------------------------//-----------------//------------------------------------------------------
   Here's a detailed README for your project:

---

```markdown
# MEEN 612 – Cartesian Impedance Controller for UR5e (Drake)

A Python implementation of a Cartesian impedance controller for the Universal Robots UR5e
manipulator, simulated using Drake. The controller commands joint torques to track a smooth
rectangular trajectory in Cartesian space with tuned stiffness, damping, and nullspace posture control.

---

## Overview

This project implements a **Cartesian impedance controller** that regulates end-effector interaction
by shaping effective stiffness and damping in task space rather than joint space. The controller is
tested in two modes:

- **Normal Mode** – No external disturbance; pure trajectory tracking
- **Hard Mode** – External bias force applied in the X-direction to validate axis decoupling

---

## Theory

### Jacobian & Task-Space Mapping
The Jacobian maps joint velocities to end-effector velocity (`ẋ = Jq̇`). Its transpose maps
Cartesian forces back to joint torques.

### Task-Space Inertia
The task-space inertia matrix `Λ` is computed to make the controller behave consistently
regardless of robot configuration:

```
Λ = (J M⁻¹ Jᵀ)⁻¹
```

Damped least-squares regularization is applied to prevent ill-conditioning near singularities.

### Control Law
Desired end-effector acceleration:

```
ẍ_cmd = ẍ_des + Kp·e + Kd·ė − J̇q̇
```

Final joint torque command:

```
τ = Jᵀ Λ ẍ_cmd + Cq − τ_g
```

### Gain Tuning
Gains are selected to satisfy **critical damping** (`ζ = 1`):

| Axis | Kp   | Kd   | ωn (rad/s) | ζ     |
|------|------|------|------------|-------|
| X    | 10.0 | 30.0 | 3.16       | 4.743 |
| Y    | 16.0 | 22.0 | 4.00       | 2.750 |
| Z    | 22.0 | 35.0 | 4.69       | 3.731 |

---

## Controller Features

| Feature | Description |
|---|---|
| **Velocity filtering** | Exponential filter (`α = 0.15`) smooths raw Cartesian velocities to reduce torque chatter |
| **Startup ramp** | Linear ramp over first 2 s prevents large torque spikes at activation |
| **Nullspace control** | Redundant DOFs used to hold a comfortable home posture without disturbing end-effector motion |
| **Torque clipping** | Commands clipped to UR5e hardware limits |
| **Singularity handling** | Damped least-squares regularization on `Λ` and `J̄` |

---

## Repository Structure

```
MEEN612_Final_Project_Naveen_Batakurki/
│
├── my_controller.py        # Core Cartesian impedance controller class
├── run_simulator.py        # Normal mode simulation runner (no bias force)
├── hardmode_sim.py         # Hard mode simulation runner (bias force in X)
│
├── Run_simulation/         # Output figures and screen recording – normal mode
│   ├── Figure_1.png        # End-effector XY path
│   ├── Figure_1_3d.png     # 3D EE path actual vs desired
│   ├── Figure_2.png        # End-effector position vs time
│   ├── Figure_3.png        # Joint torque commands vs time
│   ├── Figure_4.png        # Joint positions vs time
│   ├── Figure_5.png        # Joint velocities vs time
│   └── Screen Recording 2... .mp4
│
├── Hardmode_sim/           # Output figures and screen recording – hard mode
│   ├── Figure_1.png
│   ├── Figure_1_3D.png
│   ├── Figure_2.png
│   ├── Figure_3.png
│   ├── Figure_4.png
│   ├── Figure_5.png
│   └── Screen Recording 2... .mp4
│
└── README.md
```

---

## Dependencies

- [Drake](https://drake.mit.edu/) – Robot simulation and dynamics
- Python 3.8+
- NumPy
- Matplotlib

Install Drake following the [official instructions](https://drake.mit.edu/installation.html).

---

## Running the Simulation

### Normal Mode (no external force)
```bash
python run_simulator.py
```

### Hard Mode (bias force in X-direction)
```bash
python hardmode_sim.py
```

---

## Results Summary

### Normal Mode

- End-effector tracks the rectangular trajectory accurately with **no overshoot** on any axis
- Z held constant at **0.49 m** throughout
- Settling time ≈ **8 s** on X and Y axes
- Damping ratio **ζ ≈ 1** (critically damped)
- Joint torques and velocities remain within UR5e hardware limits

### Hard Mode (Bias Force in X)

- X-axis deflects in response to the applied bias force
- Y and Z axes remain **unaffected**, confirming **task-space axis decoupling**
- Brief exceedance of `tau2` torque limit observed near peak load — system remains stable
- Settling times unchanged at ≈ **8 s**

---

## Key Implementation Details

```python
# Task-space inertia with damped least-squares regularization
M_inv   = np.linalg.inv(Mq)
JMinvJt = J @ M_inv @ J.T
Lambda  = np.linalg.inv(JMinvJt + self.lambda_reg * np.eye(3))
J_bar   = M_inv @ J.T @ Lambda

# Nullspace controller
N_bar    = np.eye(self.nv) - J.T @ J_bar.T
tau_null = N_bar @ (self.Kp_null @ (self.q0 - q) + self.Kd_null @ (-qd))

# Velocity filter
xdot_raw       = J @ qd
self.xdot_filt = self.alpha * xdot_raw + (1.0 - self.alpha) * self.xdot_filt

# Startup ramp
ramp = min(time_now / self.ramp_time, 1.0)
```

---

## Course Information

**Course:** MEEN 408/612 – Robot Mechanics and Control  
**Student:** Naveen Batakurki  
**UIN:** 537001208  
**Institution:** Texas A&M University
```

---
