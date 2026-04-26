# arm_lib Migration Overview

## Goal

This repository has completed the functional migration from the old C-based arm
algorithm stack to the current C++ `arm_lib` implementation.

The goal of this note is to make the repository state explicit:

- business logic has switched to `arm_lib`
- simulation has switched to `arm_lib`
- old `armk_*` files are retired
- a dirty git worktree during migration is expected until the change is
  committed

## What Is Active Now

The active implementation is the C++ tree under `User/component/arm_lib/`.

The active integration points are:

- `User/task/arm_main.cpp`
- `User/device/joint/arm_oop.hpp`
- `User/device/joint/joint.hpp`
- `User/device/joint/arm_control_types.h`
- `simulation/src/arm_sim_c_api.cpp`

If you are changing behavior, adding a solver, or fixing a bug, work in those
paths and in the current `arm_lib` tree.

## What Is Retired

The following historical paths are no longer the project authority:

- old `armk_*` files under the earlier `User/component/arm_lib/` layout
- the temporary `arm6dof` compatibility route
- the old `arm_kinematics` desktop path

These names may still appear in commit history or old notes, but they are not
the implementation used by current firmware or simulation builds.

## Why Git Status Still Looks Dirty

Until the migration is committed, `git status` can show:

- deleted legacy `armk_*` files
- added current `arm_lib/*.cpp` and `arm_lib/*.h` files
- modified integration files that switched to the new stack

That state is normal for an in-progress repository migration. The important
check is whether active build paths still reference the old implementation.
They do not.

## Documentation Rule

New documentation should describe:

- `arm_lib` as the current implementation
- `simulation/` as the current desktop verification path
- old `armk_*` and `arm6dof` names only as migration history

Do not write new docs that present the retired C implementation as an
alternative active backend.
