#pragma once

// 保存平台无关的关节 PD 控制参数。
struct PdSettings {
    double proportionalGain;
    double derivativeGain;
    double maximumEffort;
};
