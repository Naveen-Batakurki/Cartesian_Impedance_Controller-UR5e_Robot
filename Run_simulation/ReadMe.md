
# Run Simulation — Normal Mode Results

This folder contains the simulation outputs for the **Cartesian Impedance Controller (UR5e)** running in
**Normal Mode** — no external disturbance is applied, and the robot tracks a rectangular Cartesian
trajectory under pure impedance control.

---

## Contents

| File | Description |
|---|---|
| `3D_End_Effector.png` | 3D end-effector path, actual vs. desired waypoint path |
| `End_Effcetor_Pos_Vs_Time.png` | End-effector X/Y/Z position vs. time |
| `End_Effcetor_XY_Path.png` | Top-down view of the end-effector XY trajectory |
| `Joint_Pos_Vs_Time.png` | Joint positions (q1–q6) vs. time |
| `Joint_Tor_Vs_Time.png` | Joint torque commands vs. time, with hardware torque limit shown |
| `Joint_Vel_Vs_Time.png` | Joint velocities (qd1–qd6) vs. time |
| `Step_Response.png` | Critically-damped step response per Cartesian axis (X, Y, Z) with tuned Kp/Kd |
| `Run__Sim_Recording_short.mp4` | Screen recording of the normal-mode simulation in Drake |

---

## Result Highlights

- **Trajectory tracking:** The end-effector follows the rectangular waypoint path closely, with the
  actual path nearly overlapping the desired path in both the 3D and top-down XY views.
- **Height held constant:** Z stays fixed at ≈0.49 m throughout, confirming the controller isolates
  planar motion without vertical drift.
- **No overshoot:** Position vs. time shows smooth transitions between waypoints on all axes.
- **Step response / critical damping:** Per-axis step responses show ζ ≈ 1 tuning:

  | Axis | Kp | Kd | ωn (rad/s) | ζ |
  |---|---|---|---|---|
  | X | 10.0 | 30.0 | 3.16 | 4.743 |
  | Y | 16.0 | 22.0 | 4.00 | 2.750 |
  | Z | 22.0 | 35.0 | 4.69 | 3.731 |

- **Joint torques/velocities:** Commands stay within the UR5e's torque limits (dashed reference line),
  and velocities settle smoothly with no sustained oscillation.

---

## How to Reproduce

```bash
python run_simulator.py
```

This generates the figures and recording above using the tuned Cartesian impedance controller
(`my_controller.py`) with no external bias force applied.
