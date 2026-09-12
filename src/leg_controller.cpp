#include "leg_controller.h"

LegController::LegController(const mjModel* model, const std::string& legName)
    : middleBindings_(createJointBindings(
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

void LegController::initializePose(mjData* data, double middleAngle) const {
    // 两个关节互为相反角度，使末段保持竖直。
    data->qpos[middleBindings_[0].qposAddress] = middleAngle;
    data->qpos[distalBindings_[0].qposAddress] = -middleAngle;
}

void LegController::applyControl(
    mjData* data,
    double middleAngle,
    const PdSettings& settings
) const {
    // 六个实例都会调用相同方法，不复制控制公式。
    applyJointPdControl(data, middleBindings_, middleAngle, settings);
    applyJointPdControl(data, distalBindings_, -middleAngle, settings);
}

double LegController::middlePosition(const mjData* data) const {
    return data->qpos[middleBindings_[0].qposAddress];
}
