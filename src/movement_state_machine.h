#pragma once

// 表示蜘蛛当前的基础移动状态。
enum class MovementState {
    Standing,
    Moving
};

// 前后方向独立于状态，避免把每个方向扩展成一种状态。
enum class MovementDirection {
    Stopped,
    Forward,
    Backward,
    Left,
    Right,
    ForwardLeft,
    ForwardRight,
    BackwardLeft,
    BackwardRight
};

// 根据四向键输入维护站立、直向和斜向移动状态。
class MovementStateMachine {
public:
    // 默认构造后保持站立状态。
    MovementStateMachine() = default;

    // 相反方向互相抵消，两个有效方向组合成斜向移动。
    bool update(
        bool forwardPressed,
        bool backwardPressed,
        bool leftPressed,
        bool rightPressed
    );

    // 返回当前状态，供后续步态模块读取。
    MovementState state() const;

    // 返回当前移动方向，站立时方向为 Stopped。
    MovementDirection direction() const;

private:
    MovementState state_{MovementState::Standing};
    MovementDirection direction_{MovementDirection::Stopped};
};

// 返回便于日志观察的状态名称。
const char* movementStateName(MovementState state);

// 返回便于日志观察的方向名称。
const char* movementDirectionName(MovementDirection direction);
