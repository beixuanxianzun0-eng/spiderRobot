// main.cpp 只保留程序入口和每帧控制主流程。
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>

#include "joint_control.h"
#include "load_all_settings.h"
#include "movement_state_machine.h"

int main(int argc, char* argv[]) {
    // 启动参数接受 MuJoCo 模型路径和蜘蛛参数文件路径。
    if (argc != 3) {
        std::cerr << "Usage: spider_sim_cpp <model.xml> <parameters.cfg>\n";
        return 1;
    }

    try {
        // 所有模型、窗口、相机、参数和六腿实例统一在加载模块创建。
        std::unique_ptr<SimulationSettings> settings
            = loadAllSettings(argv[1], argv[2]);
        SpiderParameters& parameters = settings->parameters;

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Controls: hold Q to raise, hold E to lower.\n";
        std::cout << "Movement: W forward, S backward, release to stand.\n";
        std::cout << "time\ttarget_height\tactual_height\tleg_angle\n";

        int stepCount = 0;
        MovementStateMachine movementStateMachine;

        // 核心闭环顺序：读取输入、更新目标、控制关节、推进物理、渲染。
        while (!glfwWindowShouldClose(settings->window)) {
            const bool raisePressed
                = glfwGetKey(settings->window, GLFW_KEY_Q) == GLFW_PRESS;
            const bool lowerPressed
                = glfwGetKey(settings->window, GLFW_KEY_E) == GLFW_PRESS;
            const bool forwardPressed
                = glfwGetKey(settings->window, GLFW_KEY_W) == GLFW_PRESS;
            const bool backwardPressed
                = glfwGetKey(settings->window, GLFW_KEY_S) == GLFW_PRESS;

            // 状态机只决定当前意图，实际步态将在后续模块中消费该状态。
            if (movementStateMachine.update(forwardPressed, backwardPressed)) {
                std::cout << "Movement state: "
                          << movementStateName(movementStateMachine.state())
                          << '\n';
            }

            // Q 增大第二关节角度，E 减小角度并允许越过零进入负值。
            if (raisePressed != lowerPressed) {
                const double direction = raisePressed ? 1.0 : -1.0;
                settings->targetLegBendAngle = std::clamp(
                    settings->targetLegBendAngle
                        + direction * parameters.legBendSpeed * parameters.frameDuration,
                    parameters.minimumLegBendAngle,
                    parameters.maximumLegBendAngle
                );
            }

            // 直接使用目标角；身体高度根据腿部几何关系同步变化。
            const double targetMiddleAngle = settings->targetLegBendAngle;
            settings->targetHeight = parameters.distalLegLength
                + parameters.middleLegLength * std::sin(targetMiddleAngle)
                - parameters.baseBodyHeight;

            const double frameStartTime = settings->data->time;
            while (settings->data->time - frameStartTime < parameters.frameDuration) {
                // 身体高度和六条腿分别通过自己的模块应用控制。
                applyJointPdControl(
                    settings->data.get(),
                    settings->bodyBindings,
                    settings->targetHeight,
                    parameters.bodyPd
                );
                for (const LegController& leg : settings->legs) {
                    leg.applyControl(
                        settings->data.get(),
                        targetMiddleAngle,
                        parameters.legPd
                    );
                }

                mj_step(settings->model.get(), settings->data.get());

                // 限制日志频率，保留闭环状态检查入口。
                if (stepCount % 250 == 0) {
                    std::cout
                        << settings->data->time << '\t'
                        << settings->targetHeight << '\t'
                        << settings->data->qpos[
                            settings->bodyBindings[0].qposAddress
                        ] << '\t'
                        << settings->legs[0].middlePosition(settings->data.get())
                        << '\n';
                }
                ++stepCount;
            }

            renderFrame(*settings);
        }

        return 0;
    } catch (const std::exception& exception) {
        // 加载失败时明确输出真实原因，不创建假成功结果。
        std::cerr << exception.what() << '\n';
        return 2;
    }
}
