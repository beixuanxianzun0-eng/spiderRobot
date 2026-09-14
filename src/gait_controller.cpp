#include "gait_controller.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace
{

    constexpr double pi = 3.14159265358979323846;

    // 三角组 A 为左前、右中、左后；其余三条腿组成反相的组 B。
    constexpr std::array<double, 6> tripodPhaseOffsets{
        0.0, 0.5, 0.5, 0.0, 0.0, 0.5};

    // 六个安装角用于把世界移动方向投影到每条腿的根关节切线方向。
    constexpr std::array<double, 6> legMountAngles{
        pi / 6.0,
        -pi / 6.0,
        pi / 2.0,
        -pi / 2.0,
        5.0 * pi / 6.0,
        -5.0 * pi / 6.0};

    struct MovementVector
    {
        double forward;
        double left;
    };

    // 对角方向归一化，避免同时按两个键时步幅变大。
    MovementVector movementVector(MovementDirection direction)
    {
        constexpr double diagonal = 0.7071067811865476;
        switch (direction)
        {
            case MovementDirection::Forward:
                return {1.0, 0.0};
            case MovementDirection::Backward:
                return {-1.0, 0.0};
            case MovementDirection::Left:
                return {0.0, 1.0};
            case MovementDirection::Right:
                return {0.0, -1.0};
            case MovementDirection::ForwardLeft:
                return {diagonal, diagonal};
            case MovementDirection::ForwardRight:
                return {diagonal, -diagonal};
            case MovementDirection::BackwardLeft:
                return {-diagonal, diagonal};
            case MovementDirection::BackwardRight:
                return {-diagonal, -diagonal};
            case MovementDirection::Stopped:
                return {0.0, 0.0};
        }
        return {0.0, 0.0};
    }

} // namespace

GaitController::GaitController(
    double cycleDuration,
    double strideAngle,
    double liftAngle)
{
    configure(cycleDuration, strideAngle, liftAngle);
}

void GaitController::configure(
    double cycleDuration,
    double strideAngle,
    double liftAngle)
{
    // 动态调参仍执行基本校验，避免无效值进入每帧控制。
    cycleDuration_ = cycleDuration;
    strideAngle_ = strideAngle;
    liftAngle_ = liftAngle;
    if (cycleDuration_ <= 0.0 || strideAngle_ < 0.0 || liftAngle_ < 0.0)
    {
        throw std::invalid_argument("Gait parameters must be positive.");
    }
}

std::array<LegPose, 6> GaitController::update(
    MovementState state,
    MovementDirection direction,
    double deltaTime,
    double standingMiddleAngle,
    double standingDistalAngle)
{
    std::array<LegPose, 6> poses{};

    // 站立时六条腿回到中立姿态，并让下一次起步从完整周期开始。
    if (state == MovementState::Standing || direction == MovementDirection::Stopped)
    {
        phase_ = 0.0;
        poses.fill({0.0, standingMiddleAngle, standingDistalAngle});
        return poses;
    }

    phase_ = std::fmod(phase_ + deltaTime / cycleDuration_, 1.0);
    const MovementVector travel = movementVector(direction);

    for (std::size_t index = 0; index < poses.size(); ++index)
    {
        const double legPhase = std::fmod(
            phase_ + tripodPhaseOffsets[index],
            1.0);
        double lift = 0.0;
        double swing = 0.0;

        if (legPhase < 0.5)
        {
            // 前半周期：腿离地，从后方向前方摆
            const double t = legPhase / 0.5;

            lift = liftAngle_ * std::sin(pi * t);

            swing = -strideAngle_ + 2.0 * strideAngle_ * t;
        }
        else
        {
            // 后半周期：脚踩地，从前方向后方推
            const double t = (legPhase - 0.5) / 0.5;

            swing = strideAngle_ - 2.0 * strideAngle_ * t;
        }
        // 根关节只能沿安装方向的切线摆动，因此取移动向量在该切线上的投影。
        const double rootDirection
            = -travel.forward * std::sin(legMountAngles[index])
              + travel.left * std::cos(legMountAngles[index]);
        poses[index] = {
            rootDirection * swing,
            standingMiddleAngle + std::max(0.0, lift),
            standingDistalAngle - std::max(0.0, lift)};
    }

    return poses;
}
