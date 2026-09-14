#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <spider_interfaces/msg/movement_command.hpp>
#include <spider_interfaces/msg/runtime_tuning.hpp>
#include <std_msgs/msg/float64.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "gait_controller.h"
#include "spider_joint_names.h"
#include "spider_parameters.h"

namespace {

constexpr double COMMAND_EPSILON = 0.001;
constexpr double COMMAND_TIMEOUT_SECONDS = 0.25;

}  // namespace

class GaitNode final : public rclcpp::Node {
public:
    explicit GaitNode(const SpiderParameters& parameters)
        : rclcpp::Node("spider_gait_controller"),
          parameters_(parameters),
          gaitController_(
              parameters.gaitCycleDuration,
              parameters.gaitStrideAngle,
              parameters.gaitLiftAngle
          ),
          targetMiddleAngle_(parameters.initialLegBendAngle),
          targetDistalAngle_(parameters.initialDistalLegAngle) {
        movementSubscription_
            = create_subscription<spider_interfaces::msg::MovementCommand>(
                "/spider/movement_command",
                10,
                [this](const spider_interfaces::msg::MovementCommand& message) {
                    receiveMovement(message);
                }
            );
        bendSubscription_ = create_subscription<std_msgs::msg::Float64>(
            "/spider/leg_bend_direction",
            10,
            [this](const std_msgs::msg::Float64& message) {
                bendDirection_ = std::clamp(message.data, -1.0, 1.0);
                lastBendCommandTime_ = get_clock()->now();
            }
        );
        const rclcpp::QoS tuningQos
            = rclcpp::QoS(1).transient_local().reliable();
        tuningSubscription_
            = create_subscription<spider_interfaces::msg::RuntimeTuning>(
                "/spider/runtime_tuning",
                tuningQos,
                [this](const spider_interfaces::msg::RuntimeTuning& message) {
                    receiveTuning(message);
                }
            );
        trajectoryPublisher_
            = create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/spider/joint_trajectory",
                10
            );

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(parameters_.frameDuration)
        );
        updateTimer_ = create_wall_timer(period, [this]() { publishTrajectory(); });
        lastMovementCommandTime_ = get_clock()->now();
        lastBendCommandTime_ = get_clock()->now();
    }

private:
    // 实时参数只更新步态控制器拥有的字段，物理质量由 MuJoCo 节点处理。
    void receiveTuning(
        const spider_interfaces::msg::RuntimeTuning& message
    ) {
        if (!std::isfinite(message.gait_cycle_duration)
            || !std::isfinite(message.gait_root_joint_limit)
            || !std::isfinite(message.gait_stride_angle)
            || !std::isfinite(message.gait_lift_angle)
            || !std::isfinite(message.leg_bend_speed)
            || message.gait_cycle_duration <= 0.0
            || message.gait_root_joint_limit <= 0.0
            || message.gait_stride_angle < 0.0
            || message.gait_lift_angle < 0.0
            || message.leg_bend_speed <= 0.0) {
            RCLCPP_WARN(get_logger(), "Ignored invalid runtime tuning values.");
            return;
        }

        parameters_.gaitCycleDuration = message.gait_cycle_duration;
        parameters_.gaitRootJointLimit = message.gait_root_joint_limit;
        parameters_.gaitStrideAngle = std::min(
            message.gait_stride_angle,
            message.gait_root_joint_limit
        );
        parameters_.gaitLiftAngle = std::min(
            message.gait_lift_angle,
            parameters_.initialLegBendAngle - parameters_.minimumLegBendAngle
        );
        parameters_.legBendSpeed = message.leg_bend_speed;
        gaitController_.configure(
            parameters_.gaitCycleDuration,
            parameters_.gaitStrideAngle,
            parameters_.gaitLiftAngle
        );
    }

    void receiveMovement(
        const spider_interfaces::msg::MovementCommand& message
    ) {
        if (message.state > static_cast<std::uint8_t>(MovementState::Moving)
            || message.direction
                > static_cast<std::uint8_t>(MovementDirection::BackwardRight)) {
            RCLCPP_ERROR(get_logger(), "Ignored invalid movement command.");
            return;
        }
        movementState_ = static_cast<MovementState>(message.state);
        movementDirection_ = static_cast<MovementDirection>(message.direction);
        hasMovementCommand_ = true;
        lastMovementCommandTime_ = get_clock()->now();
    }

    void publishTrajectory() {
        const rclcpp::Time currentTime = get_clock()->now();
        // 状态节点失联时步态节点独立回到站立目标。
        if (!hasMovementCommand_
            || (currentTime - lastMovementCommandTime_).seconds()
                > COMMAND_TIMEOUT_SECONDS) {
            movementState_ = MovementState::Standing;
            movementDirection_ = MovementDirection::Stopped;
        }
        if ((currentTime - lastBendCommandTime_).seconds()
            > COMMAND_TIMEOUT_SECONDS) {
            bendDirection_ = 0.0;
        }

        if (std::abs(bendDirection_) > COMMAND_EPSILON) {
            const double previousMiddleAngle = targetMiddleAngle_;
            targetMiddleAngle_ = std::clamp(
                targetMiddleAngle_
                    + bendDirection_ * parameters_.legBendSpeed
                        * parameters_.frameDuration,
                parameters_.minimumLegBendAngle,
                parameters_.maximumLegBendAngle
            );
            targetDistalAngle_ = std::clamp(
                targetDistalAngle_ - (targetMiddleAngle_ - previousMiddleAngle),
                parameters_.minimumDistalLegAngle,
                parameters_.maximumDistalLegAngle
            );
        }

        const std::array<LegPose, 6> poses = gaitController_.update(
            movementState_,
            movementDirection_,
            parameters_.frameDuration,
            targetMiddleAngle_,
            targetDistalAngle_
        );

        trajectory_msgs::msg::JointTrajectory trajectory;
        trajectory.joint_names.assign(
            spiderJointNames().begin(),
            spiderJointNames().end()
        );
        trajectory.points.resize(1);
        trajectory.points[0].positions.reserve(spiderJointNames().size());
        for (const LegPose& pose : poses) {
            trajectory.points[0].positions.push_back(pose.rootAngle);
            trajectory.points[0].positions.push_back(pose.middleAngle);
            trajectory.points[0].positions.push_back(pose.distalAngle);
        }
        trajectory.points[0].time_from_start.nanosec = static_cast<std::uint32_t>(
            parameters_.frameDuration * 1'000'000'000.0
        );
        trajectoryPublisher_->publish(trajectory);
    }

    SpiderParameters parameters_;
    GaitController gaitController_;
    MovementState movementState_{MovementState::Standing};
    MovementDirection movementDirection_{MovementDirection::Stopped};
    double targetMiddleAngle_;
    double targetDistalAngle_;
    double bendDirection_{0.0};
    bool hasMovementCommand_{false};
    rclcpp::Time lastMovementCommandTime_;
    rclcpp::Time lastBendCommandTime_;
    rclcpp::Subscription<spider_interfaces::msg::MovementCommand>::SharedPtr
        movementSubscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr bendSubscription_;
    rclcpp::Subscription<spider_interfaces::msg::RuntimeTuning>::SharedPtr
        tuningSubscription_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
        trajectoryPublisher_;
    rclcpp::TimerBase::SharedPtr updateTimer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        const std::string packagePath
            = ament_index_cpp::get_package_share_directory("spider_control");
        const SpiderParameters parameters = loadSpiderParameters(
            packagePath + "/config/spider_parameters.cfg"
        );
        rclcpp::spin(std::make_shared<GaitNode>(parameters));
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        return 1;
    }
}
