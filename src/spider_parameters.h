#pragma once

#include <string>

#include "joint_control.h"

// 保存蜘蛛模型运行时需要的初始值、动作限制和控制参数。
struct SpiderParameters {
    double frameDuration;
    double baseBodyHeight;
    double middleLegLength;
    double distalLegLength;
    double initialLegBendAngle;
    double minimumLegBendAngle;
    double maximumLegBendAngle;
    double legBendSpeed;
    PdSettings bodyPd;
    PdSettings legPd;
};

// 从独立参数文件加载全部数值；缺少或非法参数时立即报错。
SpiderParameters loadSpiderParameters(const std::string& filePath);
