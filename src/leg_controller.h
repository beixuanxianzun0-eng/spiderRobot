#pragma once

#include <mujoco/mujoco.h>

#include <string>
#include <vector>

#include "joint_control.h"

// 一个实例代表一条三段腿；六条腿共用这套控制逻辑。
class LegController {
public:
    LegController(const mjModel* model, const std::string& legName);

    // 直接设置初始姿态，避免模型从错误角度启动。
    void initializePose(
        mjData* data,
        double rootAngle,
        double middleAngle,
        double distalAngle
    ) const;

    // 根部负责前后摆动，中段抬腿，末段反向旋转以保持竖直。
    void applyControl(
        mjData* data,
        double rootAngle,
        double middleAngle,
        double distalAngle,
        const PdSettings& settings
    ) const;

    // 返回中段当前角度，供主循环输出调试信息。
    double middlePosition(const mjData* data) const;

private:
    std::vector<JointBinding> rootBindings_;
    std::vector<JointBinding> middleBindings_;
    std::vector<JointBinding> distalBindings_;
};
