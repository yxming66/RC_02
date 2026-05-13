# 3pit MATLAB Simulation Harness

This folder is now specialized for validating the 3pit / 3DOF arm control
algorithm path. The old 6DOF Pieper simulation path is intentionally retired
from the main scripts.

Primary goals:
- Load the 3pit URDF model from `assets/urdf/armurdf_standard_motor`.
- Drive validation through the current C++ library under `User/robotics/arm`.
- Simulate the real control-loop shape: current joints -> FK -> frame/absolute
  target -> bounded Cartesian step -> weighted DLS yz/pitch servo -> joint
  velocity/acceleration/step/position safety -> next joint command.
- Benchmark 3pit yz/pitch IK with FK back-checks and continuity metrics.

Requirements:
- MATLAB with Robotics System Toolbox for URDF import and visualization.
- Host `g++` for the native C++ exporter build.

Quick start:
```matlab
cd matlab_sim
run("startup_arm_sim.m")

run_3pit_model_demo()
app = run_3pit_interactive_control()
results = run_3pit_control_validation(false)
summary = run_3pit_benchmark(30)
```

`run_3pit_control_validation` builds and runs the native C++ exporter under
`matlab_sim/native`. The exporter executes 3pit MoveJ, frame target resolving,
one-shot constrained IK diagnostics, and the weighted-DLS control loop using
`User/robotics/arm`.

`run_3pit_interactive_control` opens a UI that commands `dy`/`dz`/`dpitch`
targets. Cartesian-path mode follows the C++ Cartesian control loop, while
joint-path mode first reads the selected target from the C++ `cppIk` row and
then reads the C++ MoveJ trajectory to that joint solution. The red curve is
the target TCP trajectory and the blue curve is the achieved TCP trajectory.
