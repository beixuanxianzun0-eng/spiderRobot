#pragma once

#include <array>
#include <string>

// 这是所有 ROS 2 关节轨迹发布者和执行器共同使用的唯一名称顺序。
inline const std::array<std::string, 18>& spiderJointNames() {
    static const std::array<std::string, 18> names{
        "front_left_root_joint",
        "front_left_middle_joint",
        "front_left_distal_joint",
        "front_right_root_joint",
        "front_right_middle_joint",
        "front_right_distal_joint",
        "middle_left_root_joint",
        "middle_left_middle_joint",
        "middle_left_distal_joint",
        "middle_right_root_joint",
        "middle_right_middle_joint",
        "middle_right_distal_joint",
        "rear_left_root_joint",
        "rear_left_middle_joint",
        "rear_left_distal_joint",
        "rear_right_root_joint",
        "rear_right_middle_joint",
        "rear_right_distal_joint"
    };
    return names;
}
