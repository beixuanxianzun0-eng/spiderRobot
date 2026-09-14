from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 五个进程通过 ROS 2 话题连接，调参窗口关闭后不影响控制层实现。
    return LaunchDescription(
        [
            Node(
                package="spider_control",
                executable="spider_keyboard_node",
                output="screen",
            ),
            Node(
                package="spider_control",
                executable="spider_movement_state_node",
                output="screen",
            ),
            Node(
                package="spider_control",
                executable="spider_gait_node",
                output="screen",
            ),
            Node(
                package="spider_control",
                executable="spider_tuning_node",
                output="screen",
            ),
            Node(
                package="spider_mujoco",
                executable="spider_mujoco_node",
                output="screen",
            ),
        ]
    )
