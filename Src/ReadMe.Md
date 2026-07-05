
# ur_description

This folder contains the **UR5e robot description package** (URDF, meshes, and package
metadata) used by the simulation in this project.

## Setup

1. **Unzip** `ur_description.zip` into your project's root directory (same level as
   `run_simulator.py`, `hardmode_sim.py`, `my_controller.py`, etc.), so the path looks like:
your_project/
├── ur_description/
│   ├── meshes/
│   ├── urdf/
│   ├── LICENSE
│   ├── package.xml
│   └── README
├── run_simulator.py
├── my_controller.py
└── ...
2. Do **not** rename or move the extracted `ur_description/` folder — the simulation code
   references it directly:

```python
   ur_path = "./ur_description/urdf/ur5e.urdf"
   ur_pkg  = "./ur_description/package.xml"
```

## Contents

| Item | Description |
|---|---|
| `urdf/` | Robot URDF definition for the UR5e |
| `meshes/` | Visual and collision mesh files for the robot links |
| `package.xml` | ROS-style package manifest, referenced by Drake's `Parser` |
| `LICENSE` | Original license for this package (**BSD-3-Clause**) |
| `README` | Original upstream documentation for the package |

## Licensing

This package is derived from the
[ROS-Industrial `universal_robot` repository](https://github.com/ros-industrial/universal_robot/blob/melodic-devel/ur_description/package.xml)
and is licensed under **BSD-3-Clause**.

The original `LICENSE` file is included unmodified in this folder. Per the terms of
BSD-3-Clause, this notice must be preserved when redistributing the package — so please
keep the `LICENSE` file in place and do not remove or alter it.

This license applies only to the `ur_description/` package itself. All other code in this
repository (controllers, simulation scripts, plotting utilities) is original work and is
not covered by this license.
