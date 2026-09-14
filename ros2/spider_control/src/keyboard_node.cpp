#include <GLFW/glfw3.h>

#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class KeyboardNode final : public rclcpp::Node {
public:
    KeyboardNode()
        : rclcpp::Node("spider_keyboard_input") {
        movementPublisher_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10
        );
        bendPublisher_ = create_publisher<std_msgs::msg::Float64>(
            "/spider/leg_bend_direction",
            10
        );
    }

    // 输入窗口独立于 MuJoCo，因此删除仿真包后键盘控制仍可运行。
    void publishInput(GLFWwindow* window) {
        geometry_msgs::msg::Twist movement;
        movement.linear.x
            = static_cast<double>(glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
              - static_cast<double>(glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS);
        movement.linear.y
            = static_cast<double>(glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
              - static_cast<double>(glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS);
        movementPublisher_->publish(movement);

        std_msgs::msg::Float64 bend;
        bend.data
            = static_cast<double>(glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
              - static_cast<double>(glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS);
        bendPublisher_->publish(bend);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr movementPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bendPublisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    GLFWwindow* window = nullptr;
    try {
        if (!glfwInit()) {
            throw std::runtime_error("Failed to initialize GLFW keyboard input.");
        }
        // 自动化验证时隐藏输入窗口，正常运行仍显示独立键盘窗口。
        if (std::getenv("SPIDER_HEADLESS") != nullptr) {
            glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        }
        window = glfwCreateWindow(
            460,
            140,
            "Spider ROS 2 Keyboard - W/A/S/D and Q/E",
            nullptr,
            nullptr
        );
        if (window == nullptr) {
            throw std::runtime_error("Failed to create keyboard input window.");
        }

        auto node = std::make_shared<KeyboardNode>();
        while (rclcpp::ok() && !glfwWindowShouldClose(window)) {
            node->publishInput(window);
            rclcpp::spin_some(node);
            glfwPollEvents();
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }

        glfwDestroyWindow(window);
        glfwTerminate();
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        if (window != nullptr) {
            glfwDestroyWindow(window);
        }
        glfwTerminate();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        return 1;
    }
}
