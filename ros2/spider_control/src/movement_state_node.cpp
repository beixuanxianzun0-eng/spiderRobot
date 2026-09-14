#include <chrono>
#include <cstdint>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <spider_interfaces/msg/movement_command.hpp>

#include "movement_state_machine.h"

using namespace std::chrono_literals;

// 编译期锁定内部枚举与 ROS 消息契约，避免以后修改顺序导致错误动作。
static_assert(
    static_cast<std::uint8_t>(MovementState::Standing)
        == spider_interfaces::msg::MovementCommand::STANDING
);
static_assert(
    static_cast<std::uint8_t>(MovementDirection::BackwardRight)
        == spider_interfaces::msg::MovementCommand::BACKWARD_RIGHT
);

class MovementStateNode final : public rclcpp::Node {
public:
    MovementStateNode()
        : rclcpp::Node("spider_movement_state") {
        movementSubscription_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            [this](const geometry_msgs::msg::Twist& message) {
                receiveMovement(message);
            }
        );
        commandPublisher_
            = create_publisher<spider_interfaces::msg::MovementCommand>(
                "/spider/movement_command",
                10
            );
        publishTimer_ = create_wall_timer(20ms, [this]() { publishState(); });
        lastCommandTime_ = get_clock()->now();
    }

private:
    static constexpr double COMMAND_EPSILON = 0.001;
    static constexpr double COMMAND_TIMEOUT_SECONDS = 0.25;

    void receiveMovement(const geometry_msgs::msg::Twist& message) {
        forwardPressed_ = message.linear.x > COMMAND_EPSILON;
        backwardPressed_ = message.linear.x < -COMMAND_EPSILON;
        leftPressed_ = message.linear.y > COMMAND_EPSILON;
        rightPressed_ = message.linear.y < -COMMAND_EPSILON;
        hasCommand_ = true;
        lastCommandTime_ = get_clock()->now();
    }

    void publishState() {
        // 输入节点失联后强制进入站立状态，避免真机持续移动。
        if (!hasCommand_
            || (get_clock()->now() - lastCommandTime_).seconds()
                > COMMAND_TIMEOUT_SECONDS) {
            forwardPressed_ = false;
            backwardPressed_ = false;
            leftPressed_ = false;
            rightPressed_ = false;
        }

        if (stateMachine_.update(
                forwardPressed_,
                backwardPressed_,
                leftPressed_,
                rightPressed_)) {
            RCLCPP_INFO(
                get_logger(),
                "Movement state: %s / %s",
                movementStateName(stateMachine_.state()),
                movementDirectionName(stateMachine_.direction())
            );
        }

        spider_interfaces::msg::MovementCommand command;
        command.state = static_cast<std::uint8_t>(stateMachine_.state());
        command.direction = static_cast<std::uint8_t>(stateMachine_.direction());
        commandPublisher_->publish(command);
    }

    MovementStateMachine stateMachine_;
    bool forwardPressed_{false};
    bool backwardPressed_{false};
    bool leftPressed_{false};
    bool rightPressed_{false};
    bool hasCommand_{false};
    rclcpp::Time lastCommandTime_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        movementSubscription_;
    rclcpp::Publisher<spider_interfaces::msg::MovementCommand>::SharedPtr
        commandPublisher_;
    rclcpp::TimerBase::SharedPtr publishTimer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovementStateNode>());
    rclcpp::shutdown();
    return 0;
}
