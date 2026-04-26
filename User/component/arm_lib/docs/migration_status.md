# arm_lib Migration Status

## Status

The active implementation is the current C++ `arm_lib` tree:

- `adapter/`
- `control/`
- `core/`
- `demo/`
- `dynamics/`
- `kinematics/`
- `model/`
- `parser/`
- `solver/`
- `trajectory/`

## Legacy Code

The earlier `armk_*` C implementation is retired.

That legacy layout included files such as:

- `armk_api.*`
- `armk_forward_kinematics.*`
- `armk_inverse_kinematics.*`
- `armk_gravity.*`
- `armk_joint_trajectory.*`

Those files are no longer the project authority and are not the path used by:

- STM32 firmware control
- desktop `simulation/`

## Integration Result

Current project integration uses:

- `User/device/joint/arm_oop.hpp`
- `User/device/joint/joint.hpp`
- `User/device/joint/arm_control_types.h`
- `User/task/arm_main.cpp`
- `simulation/src/arm_sim_c_api.cpp`

All of those paths now use the new `arm_lib` implementation.

`User/module/joint.hpp` is now only a compatibility shim that forwards to the
canonical runtime joint header under `User/device/joint/`.

## Git Interpretation

If the repository still shows deleted `armk_*` files and added `arm_lib/*`
files in the same diff, interpret that as migration delta, not as a mixed
runtime architecture.

The current runtime architecture is already unified on `arm_lib`.

## Practical Meaning

If you are adding features, fixing bugs, or extending manipulators, work only
against the current C++ `arm_lib` tree unless a future task explicitly asks to
recover or inspect legacy code for history.
