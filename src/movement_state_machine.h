#pragma once

// 表示蜘蛛当前的基础移动状态。
enum class MovementState {
    Standing,
    MovingForward,
    MovingBackward
};

// 根据前进和后退输入维护唯一的移动状态。
class MovementStateMachine {
public:
    // 默认构造后保持站立状态。
    MovementStateMachine() = default;

    // 同时按下或同时松开 W/S 时回到站立状态。
    bool update(bool forwardPressed, bool backwardPressed);

    // 返回当前状态，供后续步态模块读取。
    MovementState state() const;

private:
    MovementState state_{MovementState::Standing};
};

// 返回便于日志观察的状态名称。
const char* movementStateName(MovementState state);
