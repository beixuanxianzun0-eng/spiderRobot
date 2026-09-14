# ROS 2 Hexapod Controller

This repository separates platform-independent robot control from the replaceable
MuJoCo simulation adapter.

## Packages

- `spider_interfaces`: movement-state message shared between nodes.
- `spider_control`: keyboard input, movement state machine, gait controller, and core C++ library.
- `spider_mujoco`: replaceable MuJoCo physics and visualization output.

The control packages do not depend on MuJoCo. Removing `ros2/spider_mujoco`
does not affect the keyboard, state-machine, or gait packages.

## ROS 2 flow

```text
spider_keyboard_node
  -> /cmd_vel
spider_movement_state_node
  -> /spider/movement_command
spider_gait_node
  -> /spider/joint_trajectory
spider_mujoco_node or a future hardware driver
```

`/spider/leg_bend_direction` carries the independent Q/E pose command directly
from the keyboard node to the gait node.

## Build

```bash
cd ~/spider_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select spider_interfaces spider_control spider_mujoco --symlink-install
source install/setup.bash
```

## Run

Run every node together:

```bash
ros2 launch spider_mujoco spider_sim.launch.py
```

Focus the small keyboard window. Use `W/A/S/D` to move and `Q/E` to change the
shared leg bend angle. Closing the keyboard window makes the state node return
to standing after its command timeout.

VS Code provides `Debug ROS2 spider closed loop` for one-click build and
multi-process debugging. Each node also has its own debug configuration.

## Hardware replacement

A real hardware package should subscribe to `/spider/joint_trajectory`, validate
the eighteen joint names, convert radians to the motor protocol, enforce limits,
and publish encoder feedback. The state machine and gait packages stay unchanged.

MuJoCo is both a physics simulator and visualization layer; it is not part of
the platform-independent control chain.
