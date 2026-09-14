#include <QApplication>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QScrollArea>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <spider_interfaces/msg/runtime_tuning.hpp>

#include "spider_parameters.h"

class TuningNode final : public rclcpp::Node {
public:
    TuningNode()
        : rclcpp::Node("spider_runtime_tuning") {
        // 保留最后一次设置，新启动的订阅节点也能立即收到当前参数。
        const rclcpp::QoS qos = rclcpp::QoS(1).transient_local().reliable();
        publisher_ = create_publisher<spider_interfaces::msg::RuntimeTuning>(
            "/spider/runtime_tuning",
            qos
        );
    }

    // 仅在滑块变化后发布，窗口空闲时不持续占用 ROS 通信资源。
    void publishParameters(const SpiderParameters& parameters) {
        spider_interfaces::msg::RuntimeTuning message;
        message.gait_cycle_duration = parameters.gaitCycleDuration;
        message.gait_root_joint_limit = parameters.gaitRootJointLimit;
        message.gait_stride_angle = std::min(
            parameters.gaitStrideAngle,
            parameters.gaitRootJointLimit
        );
        message.gait_lift_angle = std::min(
            parameters.gaitLiftAngle,
            parameters.initialLegBendAngle - parameters.minimumLegBendAngle
        );
        message.leg_bend_speed = parameters.legBendSpeed;
        message.spider_body_mass = parameters.spiderBodyMass;
        message.leg_root_mass = parameters.legRootMass;
        message.leg_middle_mass = parameters.legMiddleMass;
        message.leg_distal_mass = parameters.legDistalMass;
        message.leg_pd_proportional_gain = parameters.legPd.proportionalGain;
        message.leg_pd_derivative_gain = parameters.legPd.derivativeGain;
        message.leg_pd_maximum_effort = parameters.legPd.maximumEffort;
        publisher_->publish(message);
    }

private:
    rclcpp::Publisher<spider_interfaces::msg::RuntimeTuning>::SharedPtr publisher_;
};

class TuningWindow final : public QWidget {
public:
    TuningWindow(
        const SpiderParameters& parameters,
        std::shared_ptr<TuningNode> node
    ) : parameters_(parameters), node_(std::move(node)) {
        setWindowTitle(QStringLiteral("蜘蛛机器人实时调参"));
        resize(620, 760);

        publishTimer_.setSingleShot(true);
        publishTimer_.setInterval(30);
        connect(
            &publishTimer_,
            &QTimer::timeout,
            this,
            [this]() { node_->publishParameters(parameters_); }
        );

        auto* content = new QWidget(this);
        auto* contentLayout = new QVBoxLayout(content);
        contentLayout->addWidget(createGaitGroup());
        contentLayout->addWidget(createMassGroup());
        contentLayout->addWidget(createPdGroup());
        contentLayout->addStretch();

        auto* scrollArea = new QScrollArea(this);
        scrollArea->setWidgetResizable(true);
        scrollArea->setWidget(content);

        auto* rootLayout = new QVBoxLayout(this);
        rootLayout->addWidget(scrollArea);

        // 窗口出现后发布一次初始值，之后只响应用户操作。
        QTimer::singleShot(
            0,
            this,
            [this]() { node_->publishParameters(parameters_); }
        );
    }

private:
    // 创建一个带中文名称、滑块和实时数值的参数行。
    void addSlider(
        QVBoxLayout* layout,
        const QString& name,
        double minimum,
        double maximum,
        double initialValue,
        int decimals,
        std::function<void(double)> applyValue
    ) {
        auto* row = new QWidget(this);
        auto* rowLayout = new QHBoxLayout(row);
        rowLayout->setContentsMargins(6, 2, 6, 2);

        auto* nameLabel = new QLabel(name, row);
        nameLabel->setMinimumWidth(220);
        auto* slider = new QSlider(Qt::Horizontal, row);
        slider->setRange(0, 1000);
        const double normalized = std::clamp(
            (initialValue - minimum) / (maximum - minimum),
            0.0,
            1.0
        );
        slider->setValue(static_cast<int>(std::round(normalized * 1000.0)));
        auto* valueLabel = new QLabel(
            QString::number(initialValue, 'f', decimals),
            row
        );
        valueLabel->setMinimumWidth(80);

        connect(
            slider,
            &QSlider::valueChanged,
            this,
            [this,
             minimum,
             maximum,
             decimals,
             valueLabel,
             applyValue = std::move(applyValue)](int position) {
                const double value
                    = minimum
                      + (maximum - minimum)
                          * static_cast<double>(position) / 1000.0;
                valueLabel->setText(QString::number(value, 'f', decimals));
                applyValue(value);
                publishTimer_.start();
            }
        );

        rowLayout->addWidget(nameLabel);
        rowLayout->addWidget(slider, 1);
        rowLayout->addWidget(valueLabel);
        layout->addWidget(row);
    }

    // 步态参数只影响控制目标，不直接接触 MuJoCo 渲染。
    QGroupBox* createGaitGroup() {
        auto* group = new QGroupBox(QStringLiteral("行走与弯腿"), this);
        auto* layout = new QVBoxLayout(group);
        addSlider(
            layout,
            QStringLiteral("步态周期（秒）"),
            0.2,
            5.0,
            parameters_.gaitCycleDuration,
            2,
            [this](double value) { parameters_.gaitCycleDuration = value; }
        );
        addSlider(
            layout,
            QStringLiteral("根关节最大摆角（弧度）"),
            0.05,
            1.5,
            parameters_.gaitRootJointLimit,
            3,
            [this](double value) { parameters_.gaitRootJointLimit = value; }
        );
        addSlider(
            layout,
            QStringLiteral("每一步跨度（弧度）"),
            0.0,
            1.5,
            parameters_.gaitStrideAngle,
            3,
            [this](double value) { parameters_.gaitStrideAngle = value; }
        );
        addSlider(
            layout,
            QStringLiteral("抬腿幅度（弧度）"),
            0.0,
            1.5,
            parameters_.gaitLiftAngle,
            3,
            [this](double value) { parameters_.gaitLiftAngle = value; }
        );
        addSlider(
            layout,
            QStringLiteral("Q/E 弯腿速度（弧度/秒）"),
            0.05,
            3.0,
            parameters_.legBendSpeed,
            2,
            [this](double value) { parameters_.legBendSpeed = value; }
        );
        return group;
    }

    // 四个质量参数会同步到身体和六条腿的 MuJoCo 刚体。
    QGroupBox* createMassGroup() {
        auto* group = new QGroupBox(QStringLiteral("身体与腿部质量"), this);
        auto* layout = new QVBoxLayout(group);
        addSlider(
            layout,
            QStringLiteral("身体质量（千克）"),
            0.1,
            5.0,
            parameters_.spiderBodyMass,
            3,
            [this](double value) { parameters_.spiderBodyMass = value; }
        );
        addSlider(
            layout,
            QStringLiteral("每条腿根部质量（千克）"),
            0.001,
            0.2,
            parameters_.legRootMass,
            4,
            [this](double value) { parameters_.legRootMass = value; }
        );
        addSlider(
            layout,
            QStringLiteral("每条腿中段质量（千克）"),
            0.001,
            0.2,
            parameters_.legMiddleMass,
            4,
            [this](double value) { parameters_.legMiddleMass = value; }
        );
        addSlider(
            layout,
            QStringLiteral("每条腿末段质量（千克）"),
            0.0001,
            0.1,
            parameters_.legDistalMass,
            4,
            [this](double value) { parameters_.legDistalMass = value; }
        );
        return group;
    }

    // PD 参数实时改变关节响应强度、阻尼和最大输出力。
    QGroupBox* createPdGroup() {
        auto* group = new QGroupBox(QStringLiteral("关节 PD 控制"), this);
        auto* layout = new QVBoxLayout(group);
        addSlider(
            layout,
            QStringLiteral("位置响应强度 Kp"),
            0.0,
            30.0,
            parameters_.legPd.proportionalGain,
            2,
            [this](double value) { parameters_.legPd.proportionalGain = value; }
        );
        addSlider(
            layout,
            QStringLiteral("速度阻尼 Kd"),
            0.0,
            10.0,
            parameters_.legPd.derivativeGain,
            2,
            [this](double value) { parameters_.legPd.derivativeGain = value; }
        );
        addSlider(
            layout,
            QStringLiteral("电机最大控制力"),
            0.01,
            5.0,
            parameters_.legPd.maximumEffort,
            3,
            [this](double value) { parameters_.legPd.maximumEffort = value; }
        );
        return group;
    }

    SpiderParameters parameters_;
    std::shared_ptr<TuningNode> node_;
    QTimer publishTimer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        QApplication application(argc, argv);
        const std::string packagePath
            = ament_index_cpp::get_package_share_directory("spider_control");
        const SpiderParameters parameters = loadSpiderParameters(
            packagePath + "/config/spider_parameters.cfg"
        );
        auto node = std::make_shared<TuningNode>();
        TuningWindow window(parameters, node);
        window.show();
        const int result = application.exec();
        rclcpp::shutdown();
        return result;
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        return 1;
    }
}
