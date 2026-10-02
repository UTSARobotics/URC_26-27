Here’s a from-scratch **C++ MuJoCo + ROS3** setup, using your `ROSE_BASE`, `rosecore`, ROS3 tools, and Mirage datatype conventions.

The example simulates a pendulum with one motor. It publishes the pendulum’s angle and angular velocity as a Mirage `List` on `/sim/state`. A command topic, `/sim/command`, is reserved for a target angle. MuJoCo exposes a C API that can be called from C++; the simulation loop uses `mj_step`. <citation src="2"></citation>

**One detail I can’t safely fill in from the information so far:** I don’t have the actual C++ ROS3 publisher/subscriber function signatures from your `test_cpp_publisher` and `test_cpp_subscriber` sources. Rather than invent API calls, I’ll show exactly where those calls belong and provide the MuJoCo and Mirage code around them. You can paste those two example source files and I can replace the marked ROS3 adapter functions with compile-ready calls.

## 1. Install MuJoCo’s C/C++ library

This source-build route installs MuJoCo under your home directory and avoids relying on a system-wide install. MuJoCo’s build uses CMake and a C++17 compiler. <citation src="2"></citation>

On Ubuntu/Debian:

```sh
sudo apt update
sudo apt install -y build-essential cmake git libglfw3-dev libx11-dev \
    libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
```

Build and install MuJoCo:

```sh
cd ~
git clone https://github.com/google-deepmind/mujoco.git
cmake -S ~/mujoco -B ~/mujoco/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$HOME/.local"
cmake --build ~/mujoco/build -j"$(nproc)"
cmake --install ~/mujoco/build
```

Check the installation:

```sh
find ~/.local -name mujoco.h -o -name 'libmujoco.so*'
```

The paths should be under `~/.local/include` and `~/.local/lib` or a versioned subdirectory. MuJoCo’s main public header is `mujoco/mujoco.h`; its C API is usable from C++ programs. <citation src="2"></citation>

## 2. Create the ROS3 project

Use the ROS3 environment you already have:

```sh
mkdir -p ~/ros3-demo/mujoco_ros3
cd ~/ros3-demo/mujoco_ros3
source ~/ros3-demo/setup-ros3.sh
```

Set the project layout:

```text
mujoco_ros3/
  pendulum.xml
  main.cpp
  Makefile
```

## 3. Create the pendulum model

Save as `pendulum.xml`:

```xml
<mujoco model="ros3_pendulum">
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <worldbody>
    <light pos="0 0 3"/>
    <geom name="floor" type="plane" size="3 3 0.1"
          rgba="0.8 0.8 0.8 1"/>

    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.05"/>
      <geom name="rod" type="capsule" fromto="0 0 0 0 0 -0.6"
            size="0.04" mass="1" rgba="0.2 0.4 0.9 1"/>
      <geom name="bob" type="sphere" pos="0 0 -0.6"
            size="0.12" mass="0.5" rgba="0.9 0.3 0.2 1"/>
    </body>
  </worldbody>

  <actuator>
    <motor name="hinge_motor" joint="hinge" gear="2"/>
  </actuator>
</mujoco>
```

## 4. Add the simulation and Mirage encoding

Save as `main.cpp`. The MuJoCo load/step and Mirage encoding are concrete. The two ROS3 adapter functions are intentionally left as integration points until you provide the exact C++ API from your examples.

```cpp
#include <mujoco/mujoco.h>
#include <mirage_embedded.h>
#include <ros3.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

// Topic names are within the active ROSE_BASE namespace.
static constexpr const char* STATE_TOPIC = "/sim/state";
static constexpr const char* COMMAND_TOPIC = "/sim/command";

// Mirage envelope:
//   List[simulation_time_seconds, joint_angle_radians,
//        joint_velocity_radians_per_second]
//
// TODO: Implement these using the exact ROS3 C++ API in your examples.
struct Ros3Bridge {
    // Construct ROS3 node, publisher on STATE_TOPIC, and subscriber
    // on COMMAND_TOPIC here, using the API from test_cpp_publisher/subscriber.

    bool try_read_target(double& target_angle_radians) {
        // TODO: Read a Mirage message from COMMAND_TOPIC.
        // Decode its List and read the target angle as f64.
        // Return true if a new command was received, otherwise false.
        (void)target_angle_radians;
        return false;
    }

    void publish_state(const void* bytes, int size) {
        // TODO: Publish `size` Mirage bytes to STATE_TOPIC using ROS3.
        (void)bytes;
        (void)size;
    }
};

static std::vector<uint8_t> encode_state(double sim_time,
                                         double angle,
                                         double velocity) {
    // Mirage buffer holds a List with three f64 values.
    uint8_t buffer[128]{};
    mirage_msg msg{};
    mirage_init(&msg, buffer, static_cast<int16_t>(sizeof(buffer)));

    int16_t ret = 0;
    ret |= mirage_write_start(&msg);
    ret |= mirage_write_fn(&msg, "List", -1, 3);
    ret |= mirage_write_f64(&msg, sim_time);
    ret |= mirage_write_f64(&msg, angle);
    ret |= mirage_write_f64(&msg, velocity);

    if (ret < 0) {
        throw std::runtime_error("Failed to encode Mirage state");
    }

    return std::vector<uint8_t>(msg.data, msg.data + msg.length);
}

int main(int argc, char** argv) {
    const char* model_path = (argc > 1) ? argv[1] : "pendulum.xml";

    char error[1024]{};
    mjModel* model = mj_loadXML(model_path, nullptr, error, sizeof(error));
    if (!model) {
        std::fprintf(stderr, "Could not load model: %s\n", error);
        return 1;
    }

    mjData* data = mj_makeData(model);
    if (!data) {
        std::fprintf(stderr, "Could not allocate MuJoCo data\n");
        mj_deleteModel(model);
        return 1;
    }

    // Start the pendulum displaced from its resting angle.
    data->qpos[0] = 0.5;
    mj_forward(model, data);

    Ros3Bridge ros;
    double target_angle = 0.0;
    double last_publish_time = -1.0;
    constexpr double publish_period = 1.0 / 50.0;

    std::puts("MuJoCo running. Publishing /sim/state at about 50 Hz.");
    std::puts("Expected command: Mirage List[f64 target_angle_radians].");

    while (true) {
        const auto wall_start = std::chrono::steady_clock::now();

        double new_target = 0.0;
        if (ros.try_read_target(new_target) && std::isfinite(new_target)) {
            target_angle = new_target;
        }

        // Simple PD controller. MuJoCo actuator input is torque-like here.
        const double angle = data->qpos[0];
        const double velocity = data->qvel[0];
        double torque = 10.0 * (target_angle - angle) - 1.0 * velocity;
        torque = std::clamp(torque, -2.0, 2.0);
        data->ctrl[0] = torque;

        mj_step(model, data);

        if (last_publish_time < 0.0 ||
            data->time - last_publish_time >= publish_period) {
            auto bytes = encode_state(data->time, data->qpos[0], data->qvel[0]);
            ros.publish_state(bytes.data(), static_cast<int>(bytes.size()));
            last_publish_time = data->time;
        }

        // Run approximately at real-time speed based on the model timestep.
        const auto elapsed = std::chrono::steady_clock::now() - wall_start;
        const auto step_duration =
            std::chrono::duration<double>(model->opt.timestep);
        if (elapsed < step_duration) {
            std::this_thread::sleep_for(step_duration - elapsed);
        }
    }

    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}
```

Add these standard headers near the top as well; they’re needed by the code above:

```cpp
#include <algorithm>
#include <cstdint>
#include <stdexcept>
```

This first pass is headless: it runs the physics and publishes state without opening a viewer. That keeps the ROS3 integration separate from rendering. Once the loop works, add MuJoCo’s viewer as a separate step.

## 5. Add the Makefile

Save as `Makefile`:

```make
CXX ?= c++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra

MUJOCO_PREFIX ?= $(HOME)/.local
ROS3_PREFIX ?= /usr/local

TARGET := build/mujoco_ros3
SOURCE := main.cpp

all: $(TARGET)

$(TARGET): $(SOURCE)
	mkdir -p build
	$(CXX) $(CXXFLAGS) \
	  -I$(ROS3_PREFIX)/include \
	  -I$(MUJOCO_PREFIX)/include \
	  $(SOURCE) \
	  -L$(ROS3_PREFIX)/lib -lros3 -lmirage -lquicksand \
	  -L$(MUJOCO_PREFIX)/lib -lmujoco \
	  -Wl,-rpath,$(MUJOCO_PREFIX)/lib \
	  -o $(TARGET)

clean:
	rm -rf build

.PHONY: all clean
```

If your MuJoCo library is in a versioned subdirectory rather than `~/.local/lib`, locate it with:

```sh
find ~/.local -name 'libmujoco.so*'
```

Then adjust the `-L` and rpath paths in the Makefile to that directory.

Build:

```sh
make
```

If your ROS3 libraries are in your development workspace rather than `/usr/local`, build with the matching prefix, for example:

```sh
make ROS3_PREFIX="$HOME/ros3"
```

## 6. Start ROS3 and run the simulator

Use the same environment and namespace in every terminal.

**Terminal 1 — start the registry:**

```sh
source ~/ros3-demo/setup-ros3.sh
export ROSE_BASE=robot1
cd ~/ros3-demo/mujoco_ros3
./build/rosecore
```

Use the path where your `rosecore` executable actually lives if it isn’t `./build/rosecore`.

**Terminal 2 — run MuJoCo:**

```sh
source ~/ros3-demo/setup-ros3.sh
export ROSE_BASE=robot1
cd ~/ros3-demo/mujoco_ros3
./build/mujoco_ros3 pendulum.xml
```

**Terminal 3 — inspect ROS3:**

```sh
source ~/ros3-demo/setup-ros3.sh
export ROSE_BASE=robot1

rosenode list
rosetopic list
rosetopic info /sim/state
rosetopic echo /sim/state
rosetopic hz /sim/state
```

The state payload is a Mirage `List` of three `f64` values:

```text
[simulation_time_seconds, angle_radians, angular_velocity_radians_per_second]
```

A command publisher should send a Mirage `List` with one `f64` value—the target angle in radians—to `/sim/command`. For example, `0.5` radians is about 29 degrees. The simulator code above holds its initial target until it receives a command.

## 7. Finish the ROS3 adapter

To make the program compile and communicate, implement `Ros3Bridge` using the precise API in your C++ examples:

- Create a node named something like `mujoco_sim`.
- Create a publisher for `/sim/state` with a fixed message buffer large enough for the Mirage payload.
- Create a subscriber for `/sim/command`.
- Decode the command payload as Mirage `List` containing one `f64`.
- Publish the encoded state bytes from `encode_state`.

The data layout follows the Mirage guidance you provided: ordered fields in a `List`, numeric values encoded with Mirage writers, and a stable schema per topic. Share `test_cpp_publisher.cpp` and `test_cpp_subscriber.cpp` (or their actual filenames) to get the ROS3 calls filled in against your installed API.

If you want to retain your own message framing over a **physical serial device**, use `roseserial` as described in your setup; it frames payloads with a 2-byte little-endian length. For this local MuJoCo process, publish Mirage payloads directly to ROS3 topics—don’t add that serial framing.
