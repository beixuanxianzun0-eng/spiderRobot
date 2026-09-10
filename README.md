# MuJoCo C++ starter

This project demonstrates one minimal closed-loop joint controller in MuJoCo.

## Structure

- `models/closed_loop.xml`: one joint and one motor.
- `src/main.cpp`: read state, calculate feedback, write torque, step simulation.
- `CMakeLists.txt`: build configuration.

## Build

```bash
cd /mnt/e/RobotProjects/spider_sim_cpp
cmake -S . -B ~/.cache/spider_sim_cpp-build -DCMAKE_BUILD_TYPE=Debug
cmake --build ~/.cache/spider_sim_cpp-build
```

## Run

```bash
~/.cache/spider_sim_cpp-build/spider_sim_cpp models/closed_loop.xml
```

The visualization window stays open until you close it or stop the VS Code debug session.
Focus the window, then hold `W` to raise the joint or `S` to lower it.
