// GLFW 负责创建窗口、读取键盘输入和交换画面缓冲区。
#include <GLFW/glfw3.h>
// MuJoCo 负责加载机器人模型、计算物理状态和渲染场景。
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <iomanip>
#include <iostream>
#include <memory>

namespace {

// unique_ptr 会在程序退出时自动调用 MuJoCo 的释放函数，避免内存泄漏。
using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

}  // namespace

int main(int argc, char* argv[]) {
    // argc 是参数数量，argv 是参数内容；这里要求用户必须传入一个 XML 模型路径。
    if (argc != 2) {
        std::cerr << "Usage: spider_sim_cpp <model.xml>\n";
        return 1;
    }

    // MuJoCo 加载 XML 失败时，会把具体错误写入这个字符数组。
    std::array<char, 1024> error{};

    // mj_loadXML 把 MJCF XML 文件编译成仿真使用的 mjModel。
    // argv[1] 是 launch.json 传入的 models/closed_loop.xml 路径。
    ModelPtr model(
        mj_loadXML(argv[1], nullptr, error.data(), error.size()),
        &mj_deleteModel
    );

    if (!model) {
        std::cerr << "Failed to load model: " << error.data() << '\n';
        return 2;
    }

    // mj_makeData 创建运行时状态，关节角度、速度和控制量都会保存在这里。
    DataPtr data(mj_makeData(model.get()), &mj_deleteData);
    if (!data) {
        std::cerr << "Failed to allocate simulation data.\n";
        return 3;
    }

    // nq、nv、nu 分别是位置量、速度量和电机控制量的数量。
    if (model->nq < 1 || model->nv < 1 || model->nu < 1) {
        std::cerr << "The model must contain one joint and one actuator.\n";
        return 4;
    }

    // glfwInit 初始化窗口系统；失败时无法创建可视化窗口。
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW.\n";
        return 5;
    }

    // glfwCreateWindow 创建一个 960×720 的桌面窗口。
    GLFWwindow* window = glfwCreateWindow(
        960, 720, "MuJoCo closed loop - W raise / S lower", nullptr, nullptr
    );
    if (!window) {
        std::cerr << "Failed to create the MuJoCo window.\n";
        return 6;
    }

    // 指定当前线程向这个窗口绘图。
    glfwMakeContextCurrent(window);
    // 开启垂直同步，使渲染速度大致保持在显示器刷新率。
    glfwSwapInterval(1);

    // camera 保存观察位置，option 保存显示选项。
    // scene 保存本帧要绘制的物体，context 保存 OpenGL 渲染资源。
    mjvCamera camera;
    mjvOption option;
    mjvScene scene;
    mjrContext context;

    // default 系列函数先把结构体设置为 MuJoCo 的安全默认值。
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&option);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);

    // 设置相机观察点、距离和角度，让机械腿启动后自动出现在画面中央。
    camera.lookat[0] = 0.15;
    camera.lookat[2] = 0.30;
    camera.distance = 1.20;
    camera.azimuth = 120.0;
    camera.elevation = -20.0;

    // mjv_makeScene 为场景分配空间，1000 表示最多容纳 1000 个可视物体。
    mjv_makeScene(model.get(), &scene, 1000);
    // mjr_makeContext 根据模型创建 GPU 渲染资源，并设置界面字体缩放。
    mjr_makeContext(model.get(), &context, mjFONTSCALE_150);

    // targetAngle 是控制器希望关节到达的目标角度，单位为弧度。
    double targetAngle = 0.0;
    // 比例增益越大，关节追赶目标越积极。
    constexpr double proportionalGain = 20.0;
    // 微分增益根据当前速度抑制振荡。
    constexpr double derivativeGain = 2.0;
    // 限制最大电机扭矩，避免控制器输出无限增大。
    constexpr double maximumTorque = 5.0;
    // 每次渲染推进约 1/60 秒的仿真时间。
    constexpr double frameDuration = 1.0 / 60.0;
    // 按住按键时，目标角度每秒改变 1 弧度。
    constexpr double targetSpeed = 1.0;
    // 目标角度限制在关节允许的安全范围内。
    constexpr double minimumTargetAngle = -1.4;
    constexpr double maximumTargetAngle = 1.4;

    // fixed 和 setprecision 让后面的浮点数固定显示三位小数。
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Controls: hold W to raise, hold S to lower.\n";
    std::cout << "time\ttarget\tangle\ttorque\n";

    int stepCount = 0;

    // 只要用户没有关闭窗口，程序就持续执行“输入、控制、仿真、渲染”循环。
    while (!glfwWindowShouldClose(window)) {
        // glfwGetKey 返回按键当前状态；GLFW_PRESS 表示此刻正在按住。
        const bool raisePressed = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS;
        const bool lowerPressed = glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS;

        // 当前关节轴的负角度方向是抬高；同时按下或同时松开时不改变目标。
        if (raisePressed != lowerPressed) {
            const double direction = raisePressed ? -1.0 : 1.0;
            // std::clamp 把目标限制在最小角和最大角之间。
            targetAngle = std::clamp(
                targetAngle + direction * targetSpeed * frameDuration,
                minimumTargetAngle,
                maximumTargetAngle
            );
        }

        // data->time 是 MuJoCo 当前累计的仿真时间。
        const double frameStartTime = data->time;
        // 一个画面帧内可能执行多次物理步进，直到推进约 1/60 秒。
        while (data->time - frameStartTime < frameDuration) {
            // qpos[0] 是第一个关节的位置；目标减当前位置得到位置误差。
            const double positionError = targetAngle - data->qpos[0];
            // qvel[0] 是关节速度；PD 控制器根据误差和速度计算扭矩。
            const double torque = proportionalGain * positionError
                                - derivativeGain * data->qvel[0];
            // ctrl[0] 是第一个执行器输入，并被限制在允许的最大扭矩内。
            data->ctrl[0] = std::clamp(torque, -maximumTorque, maximumTorque);

            // mj_step 执行一次动力学计算，并更新 qpos、qvel 和仿真时间。
            mj_step(model.get(), data.get());

            // 每 250 个物理步打印一次，避免终端输出过快。
            if (stepCount % 250 == 0) {
                std::cout << data->time << '\t'
                          << targetAngle << '\t'
                          << data->qpos[0] << '\t'
                          << data->ctrl[0] << '\n';
            }
            ++stepCount;
        }

        // viewport 保存窗口实际像素范围，窗口缩放后也能铺满画面。
        mjrRect viewport{0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        // mjv_updateScene 把最新物理状态转换成这一帧要绘制的场景。
        mjv_updateScene(
            model.get(), data.get(), &option, nullptr, &camera, mjCAT_ALL, &scene
        );
        // mjr_render 使用 MuJoCo 渲染器把场景画进当前 OpenGL 缓冲区。
        mjr_render(viewport, &scene, &context);
        // glfwSwapBuffers 把刚画好的后台画面显示到窗口中。
        glfwSwapBuffers(window);
        // glfwPollEvents 处理关闭窗口、键盘输入等系统事件。
        glfwPollEvents();
    }

    // 按照创建的反向顺序释放渲染资源和窗口。
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    glfwDestroyWindow(window);
    return 0;
}
