#include "gait_controller.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {

constexpr double pi = 3.14159265358979323846;

// 三角组 A 为左前、右中、左后；其余三条腿组成反相的组 B。
constexpr std::array<double, 6> tripodPhaseOffsets{
    0.0, 0.5, 0.5, 0.0, 0.0, 0.5
};

// Hexumi 六个根关节的 Z 轴同向，左右腿需要按安装角使用相反摆动符号。
constexpr std::array<double, 6> sideDirections{
    1.0, -1.0, 1.0, -1.0, 1.0, -1.0
};

}  // namespace

GaitController::GaitController(
    double cycleDuration,
    double strideAngle,
    double liftAngle
) : cycleDuration_(cycleDuration),
    strideAngle_(strideAngle),
    liftAngle_(liftAngle) {
    if (cycleDuration_ <= 0.0 || strideAngle_ < 0.0 || liftAngle_ < 0.0) {
        throw std::invalid_argument("Gait parameters must be positive.");
    }
}

std::array<LegPose, 6> GaitController::update(
    MovementState state,
    MovementDirection direction,
    double deltaTime,
    double standingMiddleAngle,
    double standingDistalAngle
) {
    std::array<LegPose, 6> poses{};

    // 站立时六条腿回到中立姿态，并让下一次起步从完整周期开始。
    if (state == MovementState::Standing
        || direction == MovementDirection::Stopped) {
        phase_ = 0.0;
        poses.fill({0.0, standingMiddleAngle, standingDistalAngle});
        return poses;
    }

    phase_ = std::fmod(phase_ + deltaTime / cycleDuration_, 1.0);
    const double travelDirection
        = direction == MovementDirection::Forward ? 1.0 : -1.0;

    for (std::size_t index = 0; index < poses.size(); ++index) {
        const double legPhase = std::fmod(
            phase_ + tripodPhaseOffsets[index],
            1.0
        );
        const double phaseAngle = 2.0 * pi * legPhase;

        // 前半周期抬腿摆动，后半周期保持支撑；两组三条腿互为反相。
        const double lift = legPhase < 0.5
            ? liftAngle_ * std::sin(phaseAngle)
            : 0.0;
        const double swing = strideAngle_ * std::sin(phaseAngle);

        poses[index] = {
            travelDirection * sideDirections[index] * swing,
            standingMiddleAngle + std::max(0.0, lift),
            standingDistalAngle - std::max(0.0, lift)
        };
    }

    return poses;
}
