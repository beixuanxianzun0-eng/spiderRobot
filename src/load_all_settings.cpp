#include "load_all_settings.h"

#include <array>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace {

// 六条腿共用同一套物理参数，仅名称不同。
const std::array<std::string, 6> LEG_NAMES{
    "front_left",
    "front_right",
    "middle_left",
    "middle_right",
    "rear_left",
    "rear_right"
};

// 从 GLFW 窗口取回当前仿真设置，避免使用全局相机状态。
SimulationSettings* getSimulationSettings(GLFWwindow* window) {
    return static_cast<SimulationSettings*>(glfwGetWindowUserPointer(window));
}

// 修改刚体质量时按相同比例缩放转动惯量，保持几何形状的惯性关系。
void setBodyMass(
    mjModel* model,
    const std::string& bodyName,
    double targetMass
) {
    const int bodyId = mj_name2id(model, mjOBJ_BODY, bodyName.c_str());
    if (bodyId < 0) {
        throw std::runtime_error("Required body was not found: " + bodyName);
    }
    const double sourceMass = model->body_mass[bodyId];
    if (sourceMass <= 0.0) {
        throw std::runtime_error("Required body has invalid mass: " + bodyName);
    }

    const double inertiaScale = targetMass / sourceMass;
    model->body_mass[bodyId] = targetMass;
    for (int axis = 0; axis < 3; ++axis) {
        model->body_inertia[3 * bodyId + axis] *= inertiaScale;
    }
}

// 记录鼠标按键和拖动起点。
void handleMouseButton(
    GLFWwindow* window,
    int,
    int,
    int
) {
    SimulationSettings* settings = getSimulationSettings(window);
    if (settings == nullptr) {
        return;
    }
    settings->mouseLeftPressed
        = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    settings->mouseMiddlePressed
        = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    settings->mouseRightPressed
        = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    glfwGetCursorPos(
        window,
        &settings->lastCursorX,
        &settings->lastCursorY
    );
}

// 左键旋转，右键平移，中键缩放；Shift 切换水平操作方向。
void handleMouseMove(GLFWwindow* window, double cursorX, double cursorY) {
    SimulationSettings* settings = getSimulationSettings(window);
    if (settings == nullptr
        || (!settings->mouseLeftPressed
            && !settings->mouseMiddlePressed
            && !settings->mouseRightPressed)) {
        return;
    }

    const double deltaX = cursorX - settings->lastCursorX;
    const double deltaY = cursorY - settings->lastCursorY;
    settings->lastCursorX = cursorX;
    settings->lastCursorY = cursorY;

    int windowWidth = 0;
    int windowHeight = 0;
    glfwGetWindowSize(window, &windowWidth, &windowHeight);
    if (windowHeight <= 0) {
        return;
    }

    const bool shiftPressed
        = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS
          || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    mjtMouse action = mjMOUSE_ZOOM;
    if (settings->mouseRightPressed) {
        action = shiftPressed ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (settings->mouseLeftPressed) {
        action = shiftPressed ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }

    mjv_moveCamera(
        settings->model.get(),
        action,
        deltaX / static_cast<double>(windowHeight),
        deltaY / static_cast<double>(windowHeight),
        &settings->camera
    );
}

// 滚轮控制镜头远近。
void handleMouseScroll(GLFWwindow* window, double, double offsetY) {
    SimulationSettings* settings = getSimulationSettings(window);
    if (settings == nullptr) {
        return;
    }
    mjv_moveCamera(
        settings->model.get(),
        mjMOUSE_ZOOM,
        0.0,
        -0.05 * offsetY,
        &settings->camera
    );
}

// 把参数文件中的关节限制写入 MuJoCo 模型，参数文件因此成为唯一运行时来源。
void setJointRange(
    mjModel* model,
    const std::string& jointName,
    double minimum,
    double maximum
) {
    const int jointId = mj_name2id(model, mjOBJ_JOINT, jointName.c_str());
    if (jointId < 0) {
        throw std::runtime_error("Required joint was not found: " + jointName);
    }
    model->jnt_range[2 * jointId] = minimum;
    model->jnt_range[2 * jointId + 1] = maximum;
}

}  // namespace

void applyModelPhysicalParameters(
    mjModel* model,
    mjData* data,
    const SpiderParameters& parameters
) {
    if (model == nullptr || data == nullptr) {
        throw std::invalid_argument("MuJoCo model and data are required.");
    }

    setBodyMass(model, "spider_body", parameters.spiderBodyMass);
    for (const std::string& legName : LEG_NAMES) {
        setBodyMass(model, legName + "_leg", parameters.legRootMass);
        setBodyMass(model, legName + "_middle", parameters.legMiddleMass);
        setBodyMass(model, legName + "_distal", parameters.legDistalMass);
        setJointRange(
            model,
            legName + "_root_joint",
            -parameters.gaitRootJointLimit,
            parameters.gaitRootJointLimit
        );
        setJointRange(
            model,
            legName + "_middle_joint",
            parameters.minimumLegBendAngle,
            parameters.maximumLegBendAngle
        );
        setJointRange(
            model,
            legName + "_distal_joint",
            parameters.minimumDistalLegAngle,
            parameters.maximumDistalLegAngle
        );
    }
    // 质量、惯量和限制改变后刷新 MuJoCo 派生常量。
    mj_setConst(model, data);
}

SimulationSettings::~SimulationSettings() {
    // 按照创建的反向顺序释放渲染和窗口资源。
    if (contextInitialized) {
        mjr_freeContext(&context);
    }
    if (sceneInitialized) {
        mjv_freeScene(&scene);
    }
    if (window != nullptr) {
        glfwDestroyWindow(window);
    }
    if (glfwInitialized) {
        glfwTerminate();
    }
}

std::unique_ptr<SimulationSettings> loadAllSettings(
    const char* modelPath,
    const char* parameterPath
) {
    auto settings = std::make_unique<SimulationSettings>();
    settings->parameters = loadSpiderParameters(parameterPath);

    // 加载 XML，并把解析错误转换成普通 C++ 异常。
    std::array<char, 1024> error{};
    settings->model.reset(
        mj_loadXML(modelPath, nullptr, error.data(), error.size())
    );
    if (!settings->model) {
        throw std::runtime_error("Failed to load model: " + std::string(error.data()));
    }

    settings->data.reset(mj_makeData(settings->model.get()));
    if (!settings->data) {
        throw std::runtime_error("Failed to allocate simulation data.");
    }
    if (settings->model->nu < 18) {
        throw std::runtime_error("The model must contain eighteen leg actuators.");
    }

    // 自由关节保存身体三轴位置和四元数姿态，不再由电机直接移动。
    const int bodyJointId = mj_name2id(
        settings->model.get(),
        mjOBJ_JOINT,
        "body_free_joint"
    );
    if (bodyJointId < 0
        || settings->model->jnt_type[bodyJointId] != mjJNT_FREE) {
        throw std::runtime_error("The model must contain body_free_joint.");
    }
    settings->bodyQposAddress = settings->model->jnt_qposadr[bodyJointId];

    // 每个名称创建一个 LegController，六条腿共享同一类实现。
    settings->legs.reserve(LEG_NAMES.size());
    for (const std::string& legName : LEG_NAMES) {
        settings->legs.emplace_back(settings->model.get(), legName);
    }
    applyModelPhysicalParameters(
        settings->model.get(),
        settings->data.get(),
        settings->parameters
    );
    // 参数文件直接给出身体高度和第二关节初始角度。
    settings->targetLegBendAngle = settings->parameters.initialLegBendAngle;
    settings->targetDistalLegAngle
        = settings->parameters.initialDistalLegAngle;
    settings->data->qpos[settings->bodyQposAddress + 2]
        = settings->parameters.initialBodyHeight;
    for (const LegController& leg : settings->legs) {
        leg.initializePose(
            settings->data.get(),
            0.0,
            settings->targetLegBendAngle,
            settings->targetDistalLegAngle
        );
    }
    mj_forward(settings->model.get(), settings->data.get());

    // 初始化 GLFW 窗口和 MuJoCo 渲染资源。
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW.");
    }
    settings->glfwInitialized = true;
    // 自动化验证时允许创建隐藏窗口，正常调试仍显示可视化界面。
    if (std::getenv("SPIDER_HEADLESS") != nullptr) {
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    }
    settings->window = glfwCreateWindow(
        960, 720, "MuJoCo Hexumi - ROS 2 output", nullptr, nullptr
    );
    if (settings->window == nullptr) {
        throw std::runtime_error("Failed to create the MuJoCo window.");
    }
    glfwMakeContextCurrent(settings->window);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(settings->window, settings.get());
    glfwSetMouseButtonCallback(settings->window, handleMouseButton);
    glfwSetCursorPosCallback(settings->window, handleMouseMove);
    glfwSetScrollCallback(settings->window, handleMouseScroll);

    mjv_defaultCamera(&settings->camera);
    mjv_defaultOption(&settings->option);
    mjv_defaultScene(&settings->scene);
    mjr_defaultContext(&settings->context);
    settings->camera.lookat[0] = 0.0;
    settings->camera.lookat[1] = 0.0;
    settings->camera.lookat[2] = 0.08;
    settings->camera.distance = 0.75;
    settings->camera.azimuth = 135.0;
    settings->camera.elevation = -25.0;

    mjv_makeScene(settings->model.get(), &settings->scene, 1000);
    settings->sceneInitialized = true;
    mjr_makeContext(settings->model.get(), &settings->context, mjFONTSCALE_150);
    settings->contextInitialized = true;

    return settings;
}

void renderFrame(SimulationSettings& settings) {
    const double bodyX = settings.data->qpos[settings.bodyQposAddress];
    const double bodyY = settings.data->qpos[settings.bodyQposAddress + 1];
    const double bodyZ = settings.data->qpos[settings.bodyQposAddress + 2];
    // 只叠加身体位移，因此鼠标平移后的观察偏移不会被每帧覆盖。
    if (settings.cameraFollowInitialized) {
        settings.camera.lookat[0] += bodyX - settings.lastBodyX;
        settings.camera.lookat[1] += bodyY - settings.lastBodyY;
        settings.camera.lookat[2] += bodyZ - settings.lastBodyZ;
    }
    settings.lastBodyX = bodyX;
    settings.lastBodyY = bodyY;
    settings.lastBodyZ = bodyZ;
    settings.cameraFollowInitialized = true;

    // 把最新物理状态转换为场景并显示在 GLFW 窗口。
    mjrRect viewport{0, 0, 0, 0};
    glfwGetFramebufferSize(settings.window, &viewport.width, &viewport.height);
    mjv_updateScene(
        settings.model.get(),
        settings.data.get(),
        &settings.option,
        nullptr,
        &settings.camera,
        mjCAT_ALL,
        &settings.scene
    );
    mjr_render(viewport, &settings.scene, &settings.context);
    glfwSwapBuffers(settings.window);
    glfwPollEvents();
}
