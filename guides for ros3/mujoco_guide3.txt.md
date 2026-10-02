# MuJoCo + ROS3 Setup Status

## Goal

Run a headless MuJoCo pendulum simulation as a ROS3 node:

- Publish simulation time, joint angle, and angular velocity on `/sim/state`.
- Receive a target angle on `/sim/command`.
- Encode messages using Mirage `List` values.

## Installed components and paths

**ROS3 development workspace**

```text
Headers:     ~/ros3/include
Libraries:   ~/ros3/build
Executables: ~/ros3/build
```

**Mirage and Quicksand development files**

```text
Mirage headers:     ~/mirage/include
Mirage libraries:  ~/mirage/build
Quicksand libraries: ~/quicksand/build
```

**MuJoCo**

```text
Headers:  ~/.local/include/mujoco/mujoco.h
Library:  ~/.local/lib/libmujoco.so
```

MuJoCo configuration initially failed because `xkbcommon` was missing; installing `libxkbcommon-dev` fixed that. The full build/install then encountered errors involving optional targets (`testspeed` and the `simulate` viewer). The headless setup does not need the viewer. MuJoCo was rebuilt in Debug mode with optional examples, simulation viewer, and tests disabled.

The installed MuJoCo library was verified to export the needed functions:

```text
mj_loadXML
mj_makeData
mj_step
```

## Project Makefile

The Makefile already contains the MuJoCo link and runtime-path flags in its compiler recipe:

```make
	  -L$(MUJOCO_PREFIX)/lib -lmujoco \
	  -Wl,-rpath,$(MUJOCO_PREFIX)/lib \
```

Do not enter these flags as standalone shell commands. From the directory containing the Makefile, build with:

```sh
make clean
make
```

A successful link of the MuJoCo application has **not yet been confirmed**.

## Starting ROS3

The `rosecore` executable is at `~/ros3/build/rosecore`, not `./build/rosecore` inside the MuJoCo project.

In Terminal 1:

```sh
source ~/ros3-demo/setup-ros3.sh
export ROSE_BASE=robot1
~/ros3/build/rosecore
```

Leave it running. Use the same `ROSE_BASE=robot1` in every terminal that runs ROS3 programs or tools.

## Confirmed ROS3 pub/sub test

Initially, starting the subscriber before the topic existed produced:

```text
Failed to create subscriber for topic 'cpp_chatter'
```

The test worked after starting the publisher first. In separate terminals, with the same setup and namespace:

```sh
# Publisher — start this first
source ~/ros3-demo/setup-ros3.sh
export ROSE_BASE=robot1
~/ros3/build/test_cpp_publisher cpp_chatter 10 100
```

```sh
# Subscriber
source ~/ros3-demo/setup-ros3.sh
export ROSE_BASE=robot1
~/ros3/build/test_cpp_subscriber cpp_chatter 10
```

This confirms the ROS3 test publisher/subscriber programs can communicate when the topic is created by the publisher.

## Inspecting topics

`rosetopic` is at `~/ros3/build/rosetopic`, but it is not on the shell’s `PATH`. Use its full path:

```sh
~/ros3/build/rosetopic list
~/ros3/build/rosetopic echo /cpp_chatter
```

Trying to echo `/sim/state` returned:

```text
Failed to subscribe to /sim/state (shm: /robot1%sim%state)
```

That is expected at this stage: the MuJoCo program’s ROS3 publisher method is still a placeholder, so `/sim/state` has not been created.

## Current blockers / next steps

1. Confirm the MuJoCo application builds with `make`.
2. Locate and review the ROS3 C++ test publisher and subscriber **source files**.
3. Use their actual ROS3 API calls to implement `Ros3Bridge` in `main.cpp`.
4. Rebuild and run the simulator; then check that `/sim/state` appears and that `/sim/command` can be used.

The Python virtual environment is not required to build or run the C++ simulator or ROS3 command-line tools. Attempts to activate `.venv` from the project directory failed because there is no `.venv` at either `~/ros3-demo/.venv` or `~/ros3-demo/mujoco_ros3/.venv`. The setup script has shown a `(.venv)` prompt after being sourced, so it may be handling environment activation itself.
