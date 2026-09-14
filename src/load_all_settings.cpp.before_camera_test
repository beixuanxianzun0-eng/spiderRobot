#include "load_all_settings.h"

#include <array>
#include <stdexcept>
#include <string>

namespace {

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
    const std::array<std::string, 6> legNames{
        "front_left",
        "front_right",
        "middle_left",
        "middle_right",
        "rear_left",
        "rear_right"
    };
    settings->legs.reserve(legNames.size());
    for (const std::string& legName : legNames) {
        settings->legs.emplace_back(settings->model.get(), legName);
        setJointRange(
            settings->model.get(),
            legName + "_root_joint",
            -settings->parameters.gaitRootJointLimit,
            settings->parameters.gaitRootJointLimit
        );
        setJointRange(
            settings->model.get(),
            legName + "_middle_joint",
            settings->parameters.minimumLegBendAngle,
            settings->parameters.maximumLegBendAngle
        );
        setJointRange(
            settings->model.get(),
            legName + "_distal_joint",
            settings->parameters.minimumDistalLegAngle,
            settings->parameters.maximumDistalLegAngle
        );
    }
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
    settings->window = glfwCreateWindow(
        960, 720, "MuJoCo Hexumi - W/S physical walk", nullptr, nullptr
    );
    if (settings->window == nullptr) {
        throw std::runtime_error("Failed to create the MuJoCo window.");
    }
    glfwMakeContextCurrent(settings->window);
    glfwSwapInterval(1);

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
    // 相机跟随自由身体，机器人移动后仍保持在画面中心。
    settings.camera.lookat[0]
        = settings.data->qpos[settings.bodyQposAddress];
    settings.camera.lookat[1]
        = settings.data->qpos[settings.bodyQposAddress + 1];
    settings.camera.lookat[2]
        = settings.data->qpos[settings.bodyQposAddress + 2];

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
