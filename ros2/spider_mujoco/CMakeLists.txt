cmake_minimum_required(VERSION 3.20)
project(spider_mujoco)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(MUJOCO_ROOT "$ENV{HOME}/.local/mujoco/current" CACHE PATH "MuJoCo SDK root")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(glfw3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spider_control REQUIRED)
find_package(spider_interfaces REQUIRED)
find_package(trajectory_msgs REQUIRED)
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

# colcon 使用符号链接时先解析真实目录，确保仿真源文件路径稳定。
get_filename_component(SPIDER_PACKAGE_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
get_filename_component(SPIDER_PROJECT_ROOT "${SPIDER_PACKAGE_SOURCE}/../.." ABSOLUTE)

# 此目标是可删除的仿真执行器，不被控制核心反向依赖。
add_executable(spider_mujoco_node
    src/mujoco_node.cpp
    "${SPIDER_PROJECT_ROOT}/src/joint_control.cpp"
    "${SPIDER_PROJECT_ROOT}/src/leg_controller.cpp"
    "${SPIDER_PROJECT_ROOT}/src/load_all_settings.cpp"
)
target_include_directories(spider_mujoco_node PRIVATE
    "${SPIDER_PROJECT_ROOT}/src"
    "${MUJOCO_INCLUDE_DIR}"
)
target_link_libraries(spider_mujoco_node "${MUJOCO_LIBRARY}" glfw)
ament_target_dependencies(
    spider_mujoco_node
    ament_index_cpp
    rclcpp
    spider_control
    spider_interfaces
    trajectory_msgs
)
set_target_properties(
    spider_mujoco_node
    PROPERTIES
        BUILD_RPATH "${MUJOCO_ROOT}/lib"
        INSTALL_RPATH "${MUJOCO_ROOT}/lib"
)

install(TARGETS spider_mujoco_node DESTINATION lib/${PROJECT_NAME})
install(
    FILES "${SPIDER_PROJECT_ROOT}/models/closed_loop.xml"
    DESTINATION share/${PROJECT_NAME}/models
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
