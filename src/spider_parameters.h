#pragma once

#include <string>

#include "pd_settings.h"

// 保存蜘蛛模型运行时需要的初始值、动作限制和控制参数。
struct SpiderParameters {
    double frameDuration;
    double initialBodyHeight;
    double middleLegLength;
    double distalLegLength;
    double spiderBodyMass;
    double legRootMass;
    double legMiddleMass;
    double legDistalMass;
    double initialLegBendAngle;
    double minimumLegBendAngle;
    double maximumLegBendAngle;
    double initialDistalLegAngle;
    double minimumDistalLegAngle;
    double maximumDistalLegAngle;
    double legBendSpeed;
    double gaitCycleDuration;
    double gaitRootJointLimit;
    double gaitStrideAngle;
    double gaitLiftAngle;
    PdSettings legPd;
};

// 从独立参数文件加载全部数值；缺少或非法参数时立即报错。
SpiderParameters loadSpiderParameters(const std::string& filePath);
