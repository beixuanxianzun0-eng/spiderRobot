#pragma once

#include <mujoco/mujoco.h>

#include <string>
#include <vector>

// 保存一个关节及其电机在 MuJoCo 数组中的位置。
struct JointBinding {
    int qposAddress;
    int dofAddress;
    int actuatorId;
};

// 保存一组可复用的 PD 控制参数。
struct PdSettings {
    double proportionalGain;
    double derivativeGain;
    double maximumEffort;
};

// 根据 XML 名称解析关节和电机，避免 main.cpp 依赖固定数组编号。
std::vector<JointBinding> createJointBindings(
    const mjModel* model,
    const std::vector<std::string>& jointNames,
    const std::vector<std::string>& actuatorNames
);

// 根据身体高度计算中段腿的下弯角度，使竖直末段的足端保持在地面。
double calculateLegBendAngle(
    double bodyHeight,
    double middleLength,
    double distalLength
);

// 对所有绑定关节应用相同目标位置的 PD 控制，旋转和滑动关节均可使用。
void applyJointPdControl(
    mjData* data,
    const std::vector<JointBinding>& bindings,
    double targetPosition,
    const PdSettings& settings
);
