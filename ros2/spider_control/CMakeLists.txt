cmake_minimum_required(VERSION 3.20)
project(spider_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(glfw3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spider_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# colcon 使用符号链接时先解析真实目录，确保共享核心路径稳定。
get_filename_component(SPIDER_PACKAGE_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
get_filename_component(SPIDER_PROJECT_ROOT "${SPIDER_PACKAGE_SOURCE}/../.." ABSOLUTE)

# 核心库不依赖 MuJoCo、GLFW 或 ROS 2，便于单独测试和迁移到真机。
add_library(spider_control_core STATIC
    "${SPIDER_PROJECT_ROOT}/src/gait_controller.cpp"
    "${SPIDER_PROJECT_ROOT}/src/movement_state_machine.cpp"
    "${SPIDER_PROJECT_ROOT}/src/spider_parameters.cpp"
)
target_include_directories(spider_control_core PUBLIC
    $<BUILD_INTERFACE:${SPIDER_PROJECT_ROOT}/src>
    $<INSTALL_INTERFACE:include>
)

add_executable(spider_keyboard_node src/keyboard_node.cpp)
ament_target_dependencies(spider_keyboard_node geometry_msgs rclcpp std_msgs)
target_link_libraries(spider_keyboard_node glfw)

add_executable(spider_movement_state_node src/movement_state_node.cpp)
ament_target_dependencies(
    spider_movement_state_node
    geometry_msgs
    rclcpp
    spider_interfaces
)
target_link_libraries(spider_movement_state_node spider_control_core)

add_executable(spider_gait_node src/gait_node.cpp)
ament_target_dependencies(
    spider_gait_node
    ament_index_cpp
    rclcpp
    spider_interfaces
    std_msgs
    trajectory_msgs
)
target_link_libraries(spider_gait_node spider_control_core)

# 调参窗口是独立工具节点，关闭后控制核心不依赖 Qt。
add_executable(spider_tuning_node src/tuning_node.cpp)
ament_target_dependencies(
    spider_tuning_node
    ament_index_cpp
    rclcpp
    spider_interfaces
)
target_link_libraries(
    spider_tuning_node
    spider_control_core
    Qt5::Widgets
)

install(
    TARGETS spider_control_core
    EXPORT spider_control_core_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(
    TARGETS
        spider_keyboard_node
        spider_movement_state_node
        spider_gait_node
        spider_tuning_node
    DESTINATION lib/${PROJECT_NAME}
)
install(
    FILES
        "${SPIDER_PROJECT_ROOT}/src/gait_controller.h"
        "${SPIDER_PROJECT_ROOT}/src/movement_state_machine.h"
        "${SPIDER_PROJECT_ROOT}/src/pd_settings.h"
        "${SPIDER_PROJECT_ROOT}/src/spider_joint_names.h"
        "${SPIDER_PROJECT_ROOT}/src/spider_parameters.h"
    DESTINATION include
)
install(
    FILES "${SPIDER_PROJECT_ROOT}/config/spider_parameters.cfg"
    DESTINATION share/${PROJECT_NAME}/config
)

ament_export_targets(spider_control_core_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(spider_control_core)
ament_package()
