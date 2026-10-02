## Updated MuJoCo + ROS3 setup status

You’ve successfully installed MuJoCo’s C++ headers and library:

```text
~/.local/include/mujoco/mujoco.h
~/.local/lib/libmujoco.so
~/.local/lib/libmujoco.so.3.14.1
```

The initial CMake build hit a missing `xkbcommon` dependency; installing `libxkbcommon-dev` fixed configuration. Building the full project then failed on an optional `testspeed` sample, so you built the `mujoco` target specifically. A full `cmake --install` next failed because the optional `simulate` viewer hadn’t been built. Reconfiguring with the optional simulate/examples/tests disabled allowed installation to complete. The viewer is not required for a headless simulation.

Your development headers and libraries are in these locations:

```text
ROS3 headers:     ~/ros3/include
ROS3 libraries:   ~/ros3/build
Mirage headers:   ~/mirage/include
Mirage libraries: ~/mirage/build
Quicksand libs:   ~/quicksand/build
MuJoCo headers:   ~/.local/include
MuJoCo libraries: ~/.local/lib
```

You do **not** need to run `make` from inside the Python venv specifically; this is a C++ build. You do need to source the ROS3 setup script and use the correct include and library paths.

## Current build issue

The build now finds `ros3.h` and gets as far as linking, but fails to resolve MuJoCo functions such as `mj_loadXML`, `mj_makeData`, and `mj_step`. The next diagnostic is:

```sh
nm -D "$HOME/.local/lib/libmujoco.so" | grep -E 'mj_loadXML|mj_makeData|mj_step'
```

If that prints no matches, inspect which file the symlink points to:

```sh
readlink -f "$HOME/.local/lib/libmujoco.so"
```

If it prints the symbols, save that output. Those checks will distinguish a library/install problem from a link-command or header problem. **The program has not successfully linked yet.**

## ROS3 bridge code is still incomplete

The earlier `main.cpp` contains placeholder ROS3 publisher/subscriber methods. Even after fixing the MuJoCo linking error, those placeholders must be implemented with the actual API used by your C++ examples. The intended first test is a headless MuJoCo pendulum that publishes `/sim/state` and receives `/sim/command`, using Mirage `List` messages. The Arduino-specific `mirage_embedded.h` should not be used in the desktop C++ program; your desktop header is `mirage.h`.

Your namespace workflow remains:

```sh
# Terminal 1
source ~/ros3-demo/setup-ros3.sh
export ROSE_BASE=robot1
./build/rosecore

# Other terminals
source ~/ros3-demo/setup-ros3.sh
export ROSE_BASE=robot1
```

Once the node builds and runs, use `rosetopic list`, `rosetopic echo /sim/state`, and `rosetopic hz /sim/state` to check it.
