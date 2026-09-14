#include <mujoco/mujoco.h>

#include <array>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <spider_interfaces/msg/runtime_tuning.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "gait_controller.h"
#include "load_all_settings.h"
#include "spider_joint_names.h"

class MujocoNode final : public rclcpp::Node {
public:
    MujocoNode(
        mjModel* model,
        mjData* data,
        std::vector<LegController>& legs,
        const SpiderParameters& parameters
    ) : rclcpp::Node("spider_mujoco_output"),
        model_(model),
        data_(data),
        legs_(legs),
        parameters_(parameters),
        pdSettings_(parameters.legPd) {
        if (model_ == nullptr
            || data_ == nullptr
            || legs_.size() != targetPoses_.size()) {
            throw std::invalid_argument(
                "MuJoCo output requires data and exactly six legs."
            );
        }
        targetPoses_.fill({
            0.0,
            parameters_.initialLegBendAngle,
            parameters_.initialDistalLegAngle
        });
        trajectorySubscription_
            = create_subscription<trajectory_msgs::msg::JointTrajectory>(
                "/spider/joint_trajectory",
                10,
                [this](const trajectory_msgs::msg::JointTrajectory& message) {
                    receiveTrajectory(message);
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
    }

    // 只有此适配节点接触 MuJoCo 电机，控制节点不依赖仿真 API。
    void applyCommand() {
        for (std::size_t index = 0; index < legs_.size(); ++index) {
            legs_[index].applyControl(
                data_,
                targetPoses_[index].rootAngle,
                targetPoses_[index].middleAngle,
                targetPoses_[index].distalAngle,
                pdSettings_
            );
        }
    }

private:
    // 物理参数在仿真线程内更新，避免和 MuJoCo 步进并发写模型。
    void receiveTuning(
        const spider_interfaces::msg::RuntimeTuning& message
    ) {
        if (!std::isfinite(message.gait_root_joint_limit)
            || !std::isfinite(message.spider_body_mass)
            || !std::isfinite(message.leg_root_mass)
            || !std::isfinite(message.leg_middle_mass)
            || !std::isfinite(message.leg_distal_mass)
            || !std::isfinite(message.leg_pd_proportional_gain)
            || !std::isfinite(message.leg_pd_derivative_gain)
            || !std::isfinite(message.leg_pd_maximum_effort)
            || message.gait_root_joint_limit <= 0.0
            || message.spider_body_mass <= 0.0
            || message.leg_root_mass <= 0.0
            || message.leg_middle_mass <= 0.0
            || message.leg_distal_mass <= 0.0
            || message.leg_pd_proportional_gain < 0.0
            || message.leg_pd_derivative_gain < 0.0
            || message.leg_pd_maximum_effort <= 0.0) {
            RCLCPP_WARN(get_logger(), "Ignored invalid runtime tuning values.");
            return;
        }

        parameters_.gaitRootJointLimit = message.gait_root_joint_limit;
        parameters_.spiderBodyMass = message.spider_body_mass;
        parameters_.legRootMass = message.leg_root_mass;
        parameters_.legMiddleMass = message.leg_middle_mass;
        parameters_.legDistalMass = message.leg_distal_mass;
        parameters_.legPd.proportionalGain
            = message.leg_pd_proportional_gain;
        parameters_.legPd.derivativeGain
            = message.leg_pd_derivative_gain;
        parameters_.legPd.maximumEffort
            = message.leg_pd_maximum_effort;
        pdSettings_ = parameters_.legPd;
        applyModelPhysicalParameters(model_, data_, parameters_);
    }

    void receiveTrajectory(
        const trajectory_msgs::msg::JointTrajectory& message
    ) {
        const std::vector<std::string> expectedNames(
            spiderJointNames().begin(),
            spiderJointNames().end()
        );
        if (message.joint_names != expectedNames
            || message.points.empty()
            || message.points[0].positions.size()
                != spiderJointNames().size()) {
            RCLCPP_ERROR(
                get_logger(),
                "Ignored malformed spider joint trajectory."
            );
            return;
        }

        for (std::size_t index = 0; index < targetPoses_.size(); ++index) {
            const std::size_t offset = index * 3;
            targetPoses_[index] = {
                message.points[0].positions[offset],
                message.points[0].positions[offset + 1],
                message.points[0].positions[offset + 2]
            };
        }
    }

    mjModel* model_;
    mjData* data_;
    std::vector<LegController>& legs_;
    SpiderParameters parameters_;
    PdSettings pdSettings_;
    std::array<LegPose, 6> targetPoses_{};
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
        trajectorySubscription_;
    rclcpp::Subscription<spider_interfaces::msg::RuntimeTuning>::SharedPtr
        tuningSubscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        const std::string mujocoShare
            = ament_index_cpp::get_package_share_directory("spider_mujoco");
        const std::string controlShare
            = ament_index_cpp::get_package_share_directory("spider_control");
        std::unique_ptr<SimulationSettings> settings = loadAllSettings(
            (mujocoShare + "/models/closed_loop.xml").c_str(),
            (controlShare + "/config/spider_parameters.cfg").c_str()
        );
        auto node = std::make_shared<MujocoNode>(
            settings->model.get(),
            settings->data.get(),
            settings->legs,
            settings->parameters
        );

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "time\tbody_x\tbody_y\tbody_height\tleg_angle\n";
        int stepCount = 0;
        while (rclcpp::ok() && !glfwWindowShouldClose(settings->window)) {
            rclcpp::spin_some(node);
            const double frameStartTime = settings->data->time;
            while (settings->data->time - frameStartTime
                   < settings->parameters.frameDuration) {
                node->applyCommand();
                mj_step(settings->model.get(), settings->data.get());
                if (stepCount % 250 == 0) {
                    std::cout
                        << settings->data->time << '\t'
                        << settings->data->qpos[settings->bodyQposAddress] << '\t'
                        << settings->data->qpos[settings->bodyQposAddress + 1] << '\t'
                        << settings->data->qpos[settings->bodyQposAddress + 2] << '\t'
                        << settings->legs[0].middlePosition(settings->data.get())
                        << '\n';
                }
                ++stepCount;
            }
            renderFrame(*settings);
        }

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
