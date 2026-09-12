#include "spider_parameters.h"

#include <fstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace {

// 去除参数名称和值两侧的空白字符。
std::string trim(const std::string& text) {
    const std::size_t first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const std::size_t last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

// 读取必需参数，避免缺少配置时静默使用错误数值。
double requireDouble(
    const std::unordered_map<std::string, double>& values,
    const std::string& name
) {
    const auto iterator = values.find(name);
    if (iterator == values.end()) {
        throw std::runtime_error("Missing spider parameter: " + name);
    }
    return iterator->second;
}

}  // namespace

SpiderParameters loadSpiderParameters(const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file) {
        throw std::runtime_error("Failed to open spider parameter file: " + filePath);
    }

    std::unordered_map<std::string, double> values;
    std::string line;
    int lineNumber = 0;
    while (std::getline(file, line)) {
        ++lineNumber;
        const std::string cleaned = trim(line);
        if (cleaned.empty() || cleaned.front() == '#') {
            continue;
        }

        const std::size_t separator = cleaned.find('=');
        if (separator == std::string::npos) {
            throw std::runtime_error(
                "Invalid spider parameter at line " + std::to_string(lineNumber)
            );
        }

        const std::string name = trim(cleaned.substr(0, separator));
        const std::string valueText = trim(cleaned.substr(separator + 1));
        try {
            std::size_t parsedLength = 0;
            const double value = std::stod(valueText, &parsedLength);
            if (parsedLength != valueText.size()) {
                throw std::invalid_argument("Trailing characters");
            }
            values[name] = value;
        } catch (const std::exception&) {
            throw std::runtime_error(
                "Invalid numeric spider parameter at line "
                + std::to_string(lineNumber)
            );
        }
    }

    SpiderParameters parameters{
        requireDouble(values, "frame_duration"),
        requireDouble(values, "base_body_height"),
        requireDouble(values, "middle_leg_length"),
        requireDouble(values, "distal_leg_length"),
        requireDouble(values, "initial_leg_bend_angle"),
        requireDouble(values, "minimum_leg_bend_angle"),
        requireDouble(values, "maximum_leg_bend_angle"),
        requireDouble(values, "leg_bend_speed"),
        {
            requireDouble(values, "body_pd_proportional_gain"),
            requireDouble(values, "body_pd_derivative_gain"),
            requireDouble(values, "body_pd_maximum_effort")
        },
        {
            requireDouble(values, "leg_pd_proportional_gain"),
            requireDouble(values, "leg_pd_derivative_gain"),
            requireDouble(values, "leg_pd_maximum_effort")
        }
    };

    // 检查相互关联的限制，避免运行时出现反向范围。
    // 当前几何关系允许关节从负九十度连续转到正九十度。
    constexpr double maximumCalculatedBendAngle = 1.5707963267948966;
    if (parameters.minimumLegBendAngle > parameters.maximumLegBendAngle
        || parameters.initialLegBendAngle < parameters.minimumLegBendAngle
        || parameters.initialLegBendAngle > parameters.maximumLegBendAngle
        || parameters.minimumLegBendAngle < -maximumCalculatedBendAngle
        || parameters.maximumLegBendAngle > maximumCalculatedBendAngle) {
        throw std::runtime_error(
            "Leg bend angles must stay ordered between -pi/2 and pi/2."
        );
    }
    if (parameters.legBendSpeed <= 0.0
        || parameters.frameDuration <= 0.0
        || parameters.middleLegLength <= 0.0
        || parameters.distalLegLength < 0.0) {
        throw std::runtime_error("Spider speed, timing, and lengths must be valid.");
    }

    return parameters;
}
