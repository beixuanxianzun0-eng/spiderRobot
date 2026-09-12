# MuJoCo C++ starter

This project demonstrates one minimal closed-loop body-height controller in MuJoCo.

## Structure

- `models/closed_loop.xml`: a body, six articulated three-segment legs, a ground plane, and thirteen motors.
- `src/main.cpp`: the input-control-simulation-render core loop.
- `src/load_all_settings.cpp`: load the model, window, camera, settings, and six leg instances.
- `src/leg_controller.cpp`: reusable control logic for one three-segment leg.
- `src/joint_control.cpp`: generic PD feedback and motor output.
- `CMakeLists.txt`: build configuration.

## Build

```bash
cd /mnt/e/RobotProjects/spider_sim_cpp
cmake -S . -B ~/.cache/spider_sim_cpp-build -DCMAKE_BUILD_TYPE=Debug
cmake --build ~/.cache/spider_sim_cpp-build
```

## Run

```bash
~/.cache/spider_sim_cpp-build/spider_sim_cpp models/closed_loop.xml config/spider_parameters.cfg
```

The visualization window stays open until you close it or stop the VS Code debug session.
Focus the window, then hold `Q` to raise the body or `E` to lower it.
`W` and `S` are currently unused. Each leg keeps its root segment horizontal,
bends its middle segment for height control, and keeps its distal segment vertical.
