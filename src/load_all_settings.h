#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <memory>
#include <vector>

#include "joint_control.h"
#include "leg_controller.h"
#include "spider_parameters.h"

// 自动释放 MuJoCo 模型和运行数据。
using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

// 集中保存启动后需要的模型、窗口、控制器和可调参数。
struct SimulationSettings {
    ModelPtr model{nullptr, &mj_deleteModel};
    DataPtr data{nullptr, &mj_deleteData};
    GLFWwindow* window{nullptr};
    mjvCamera camera{};
    mjvOption option{};
    mjvScene scene{};
    mjrContext context{};
    bool glfwInitialized{false};
    bool sceneInitialized{false};
    bool contextInitialized{false};

    std::vector<LegController> legs;
    int bodyQposAddress{-1};

    SpiderParameters parameters{};
    double targetLegBendAngle{0.0};
    double targetDistalLegAngle{0.0};

    SimulationSettings() = default;
    ~SimulationSettings();
    SimulationSettings(const SimulationSettings&) = delete;
    SimulationSettings& operator=(const SimulationSettings&) = delete;
};

// 一次完成模型、窗口、相机、关节绑定和初始姿态加载。
std::unique_ptr<SimulationSettings> loadAllSettings(
    const char* modelPath,
    const char* parameterPath
);

// 使用当前物理状态绘制一帧，并处理窗口事件。
void renderFrame(SimulationSettings& settings);
