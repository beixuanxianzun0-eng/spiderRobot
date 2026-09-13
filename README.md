# MuJoCo C++ starter

This project demonstrates a minimal modular hexapod controller in MuJoCo.

## Structure

- `models/closed_loop.xml`: a rectangular body, six mirrored three-joint legs, a ground plane, and eighteen motors.
- `src/main.cpp`: the input-control-simulation-render core loop.
- `src/load_all_settings.cpp`: load the model, window, camera, settings, and six leg instances.
- `src/leg_controller.cpp`: reusable control logic for one three-segment leg.
- `src/movement_state_machine.cpp`: standing/moving state and forward/backward direction.
- `src/gait_controller.cpp`: alternating tripod walking animation.
- `src/joint_control.cpp`: generic PD feedback and motor output.
- `config/spider_parameters.cfg`: editable pose, limit, PD, and gait parameters.
- `CMakeLists.txt`: build configuration.

## Build

```bash
cd /mnt/e/RobotProjects/spider_sim_cpp
cmake -S . -B ~/.cache/spider_sim_cpp-d-build -DCMAKE_BUILD_TYPE=Debug
cmake --build ~/.cache/spider_sim_cpp-d-build
```

## Run

```bash
~/.cache/spider_sim_cpp-d-build/spider_sim_cpp models/closed_loop.xml config/spider_parameters.cfg
```

The visualization window stays open until you close it or stop the VS Code debug session.
Focus the window, then hold `Q` or `E` to adjust the standing leg angle.
Hold `W` to walk forward or `S` to walk backward using the tripod gait.
Releasing both movement keys returns the state machine to standing.

Gravity, collision, foot friction, and a freely moving body are enabled, so the
walking gait produces physical translation instead of directly changing body coordinates.
