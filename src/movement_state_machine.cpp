#include "movement_state_machine.h"

bool MovementStateMachine::update(
    bool forwardPressed,
    bool backwardPressed
) {
    MovementState nextState = MovementState::Standing;
    if (forwardPressed != backwardPressed) {
        nextState = forwardPressed
            ? MovementState::MovingForward
            : MovementState::MovingBackward;
    }

    // 返回状态是否发生变化，避免每帧重复打印相同内容。
    const bool changed = nextState != state_;
    state_ = nextState;
    return changed;
}

MovementState MovementStateMachine::state() const {
    return state_;
}

const char* movementStateName(MovementState state) {
    switch (state) {
        case MovementState::Standing:
            return "Standing";
        case MovementState::MovingForward:
            return "MovingForward";
        case MovementState::MovingBackward:
            return "MovingBackward";
    }
    return "Unknown";
}
