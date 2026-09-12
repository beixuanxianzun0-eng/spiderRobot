# Configure the standalone MuJoCo C++ environment check.
cmake_minimum_required(VERSION 3.20)
project(spider_sim_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(MUJOCO_ROOT "$ENV{HOME}/.local/mujoco/current" CACHE PATH "MuJoCo SDK root")

find_path(
    MUJOCO_INCLUDE_DIR
    NAMES mujoco/mujoco.h
    HINTS "${MUJOCO_ROOT}/include"
    REQUIRED
)

find_library(
    MUJOCO_LIBRARY
    NAMES mujoco
    HINTS "${MUJOCO_ROOT}/lib"
    REQUIRED
)

find_package(glfw3 REQUIRED)

add_executable(
    spider_sim_cpp
    src/main.cpp
    src/joint_control.cpp
    src/leg_controller.cpp
    src/load_all_settings.cpp
    src/movement_state_machine.cpp
    src/spider_parameters.cpp
)
target_include_directories(spider_sim_cpp PRIVATE "${MUJOCO_INCLUDE_DIR}")
target_link_libraries(spider_sim_cpp PRIVATE "${MUJOCO_LIBRARY}" glfw)

# Allow the executable to find the user-local MuJoCo shared library.
set_target_properties(
    spider_sim_cpp
    PROPERTIES BUILD_RPATH "${MUJOCO_ROOT}/lib"
)
