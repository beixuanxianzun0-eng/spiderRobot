cmake_minimum_required(VERSION 3.20)
project(spider_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 只定义跨节点契约，不包含控制或仿真实现。
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/MovementCommand.msg"
    "msg/RuntimeTuning.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
