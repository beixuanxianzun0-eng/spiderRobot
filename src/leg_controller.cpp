#include "leg_controller.h"

LegController::LegController(const mjModel* model, const std::string& legName)
    : rootBindings_(createJointBindings(
          model,
          {legName + "_root_joint"},
          {legName + "_root_motor"}
      )),
      middleBindings_(createJointBindings(
          model,
          {legName + "_middle_joint"},
          {legName + "_middle_motor"}
      )),
      distalBindings_(createJointBindings(
          model,
          {legName + "_distal_joint"},
          {legName + "_distal_motor"}
      )) {
}

void LegController::initializePose(
    mjData* data,
    double rootAngle,
    double middleAngle,
    double distalAngle
) const {
    // 三个关节分别使用官方站立零位，避免把末段错误地绑成等角反向。
    data->qpos[rootBindings_[0].qposAddress] = rootAngle;
    data->qpos[middleBindings_[0].qposAddress] = middleAngle;
    data->qpos[distalBindings_[0].qposAddress] = distalAngle;
}

void LegController::applyControl(
    mjData* data,
    double rootAngle,
    double middleAngle,
    double distalAngle,
    const PdSettings& settings
) const {
    // 六个实例都会调用相同方法，不复制控制公式。
    applyJointPdControl(data, rootBindings_, rootAngle, settings);
    applyJointPdControl(data, middleBindings_, middleAngle, settings);
    applyJointPdControl(data, distalBindings_, distalAngle, settings);
}

double LegController::middlePosition(const mjData* data) const {
    return data->qpos[middleBindings_[0].qposAddress];
}
