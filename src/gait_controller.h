#pragma once

#include <array>

#include "movement_state_machine.h"

// 保存一条腿在当前动画帧需要追踪的三个关节角。
struct LegPose {
    double rootAngle;
    double middleAngle;
    double distalAngle;
};

// 生成六足交替三角步态，不直接依赖 MuJoCo。
class GaitController {
public:
    GaitController(
        double cycleDuration,
        double strideAngle,
        double liftAngle
    );

    // 根据状态、方向和时间推进动画，并返回六条腿的目标姿态。
    std::array<LegPose, 6> update(
        MovementState state,
        MovementDirection direction,
        double deltaTime,
        double standingMiddleAngle,
        double standingDistalAngle
    );

private:
    double cycleDuration_;
    double strideAngle_;
    double liftAngle_;
    double phase_{0.0};
};
