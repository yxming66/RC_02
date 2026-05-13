# robotics/arm

`User/robotics/arm` is the serial-manipulator algorithm library used by this
project. It owns arm-level kinematics, dynamics, planning, servo control,
safety checks, and the controller state machine.

This directory is now the authoritative arm algorithm implementation for the
repository. New features and bug fixes should be added here, not to any of the
retired C-era files.

## Scope

This library is designed for serial robotic arms with fixed-size template
DOF:

- `SerialChain<N>` as the central chain model
- FK / Jacobian / numeric IK
- analytic IK plugin interface
- 3R planar analytic IK plugin
- hybrid IK with numeric fallback
- task-space constraint and orientation relaxation
- joint-space and Cartesian trajectory generation
- joint servo and Cartesian servo
- DH and URDF chain loading
- gravity compensation
- joint-actuator transmission / coupling mapping

## Current Status

The library is no longer a scaffold. It is integrated into:

- STM32 firmware control path
- desktop `simulation/` shared library path

Both paths currently build successfully in this repository.

The active project entry points are:

- `User/task/arm_main.cpp`
- `User/module/arm_runtime/robotic_arm.hpp`
- `simulation/src/arm_sim_c_api.cpp`

Those paths all use the current C++ `User/robotics/arm` implementation.

## Directory Layout

- `core/`
  - common scalar helpers, limits, status, matrix aliases
- `adapter/`
  - adapter layer between arm robotics algorithms and the existing toolbox math library
- `model/`
  - `ChainJointSpec`, `Link`, `ToolFrame`, `SerialChain<N>`
- `kinematics/`
  - FK, Jacobian, pose error, numeric IK, analytic IK, task constraints
- `solver/`
  - LM / NR delta solvers, seed policies
- `parser/`
  - DH loader, URDF loader, chain builder helpers
- `trajectory/`
  - interpolation, joint trajectory, Cartesian trajectory
- `control/`
  - joint servo and Cartesian servo
- `dynamics/`
  - gravity torque model
- `tests/`
  - host-side algorithm regression checks

## Integration Notes

- Inside `arm_lib`, `Chain*` naming is now the preferred internal terminology
  for chain-model joint descriptors. Older `Joint*` aliases remain only as a
  compatibility layer during migration.
- New call sites should use `ChainJointType`, `ChainJointSpec`,
  `ChainTransmission`, and `ChainJointState` directly instead of introducing
  fresh dependencies on the legacy `Joint*` aliases.
- The model layer supports per-joint transmission mapping.
- Coupling is intended for sequentially solvable chain/belt style mappings.
- Project-level motor coupling is handled in the joint runtime layer and is
  mirrored in chain transmission where needed.

## IK Configuration Guidance

The preferred way to configure inverse kinematics is now through predefined
profiles instead of manually populating every `IKOptions` field at each call
site.

Recommended entry points:

- `kinematics::make_ik_options(IkProfile::kRobust)`
- `kinematics::make_ik_options(IkProfile::kEmbeddedSafe)`
- `kinematics::make_position_only_ik_options(target_pose)`
- `kinematics::make_constrained_ik_options(target_pose, profile, ik_profile)`

This keeps firmware, simulation, and future tools aligned on the same damping,
retry, and null-space defaults while still allowing small local overrides such
as tolerance or iteration count.

## Migration State In Git

During migration, `git status` may show two kinds of arm-related changes at the
same time:

- deleted historical `armk_*` files
- added current C++ `arm_lib/*` files

That is expected until the migration commit is created. It does not mean the
runtime path is still split across both implementations.

## What Is Not Done Yet

The following are still outside the current completion scope:

- full rigid-body dynamics
  - mass matrix
  - coriolis / centrifugal terms
  - inverse dynamics
  - forward dynamics
- general analytic IK for arbitrary manipulators
- collision checking and obstacle-aware planning
- dense general actuator-joint coupling matrix solve

## Legacy Status

The old `armk_*` files under the historical `arm_lib` layout are retired and
are no longer part of the active implementation. The current source tree under
this directory is the authoritative implementation.

If older notes, screenshots, or local experiments still mention
`arm_kinematics`, `arm6dof`, or specific `armk_*` files, treat them as
historical references only.
