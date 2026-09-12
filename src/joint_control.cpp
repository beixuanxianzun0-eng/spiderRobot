#include "joint_control.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

std::vector<JointBinding> createJointBindings(
    const mjModel* model,
    const std::vector<std::string>& jointNames,
    const std::vector<std::string>& actuatorNames
) {
    // 一个关节必须对应一个电机，否则无法建立完整控制绑定。
    if (jointNames.size() != actuatorNames.size()) {
        throw std::runtime_error("Joint and actuator counts must match.");
    }

    std::vector<JointBinding> bindings;
    bindings.reserve(jointNames.size());

    for (std::size_t index = 0; index < jointNames.size(); ++index) {
        // 通过 XML 名称查找编号，XML 调整顺序后控制关系仍然正确。
        const int jointId = mj_name2id(model, mjOBJ_JOINT, jointNames[index].c_str());
        const int actuatorId = mj_name2id(model, mjOBJ_ACTUATOR, actuatorNames[index].c_str());

        if (jointId < 0 || actuatorId < 0) {
            throw std::runtime_error("Required joint or actuator was not found in the XML model.");
        }

        bindings.push_back({
            model->jnt_qposadr[jointId],
            model->jnt_dofadr[jointId],
            actuatorId
        });
    }

    return bindings;
}

double calculateLegBendAngle(
    double bodyHeight,
    double middleLength,
    double distalLength
) {
    // 中段长度必须有效，否则无法建立高度与角度的几何关系。
    if (middleLength <= 0.0 || distalLength < 0.0) {
        throw std::invalid_argument("Leg segment lengths must be valid.");
    }

    // 身体高度等于中段的竖直投影加上竖直末段长度。
    const double sineValue = std::clamp(
        (bodyHeight - distalLength) / middleLength,
        -1.0,
        1.0
    );
    return std::asin(sineValue);
}

void applyJointPdControl(
    mjData* data,
    const std::vector<JointBinding>& bindings,
    double targetPosition,
    const PdSettings& settings
) {
    for (const JointBinding& binding : bindings) {
        // 目标位置误差产生驱动力，当前速度产生阻尼力。
        const double positionError = targetPosition - data->qpos[binding.qposAddress];
        const double effort = settings.proportionalGain * positionError
                            - settings.derivativeGain * data->qvel[binding.dofAddress];

        // 将结果写入对应电机，并限制最大控制力。
        data->ctrl[binding.actuatorId] = std::clamp(
            effort,
            -settings.maximumEffort,
            settings.maximumEffort
        );
    }
}
