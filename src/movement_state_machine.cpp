#include "movement_state_machine.h"

bool MovementStateMachine::update(
    bool forwardPressed,
    bool backwardPressed
) {
    MovementState nextState = MovementState::Standing;
    MovementDirection nextDirection = MovementDirection::Stopped;
    if (forwardPressed != backwardPressed) {
        nextState = MovementState::Moving;
        nextDirection = forwardPressed
            ? MovementDirection::Forward
            : MovementDirection::Backward;
    }

    // 返回状态是否发生变化，避免每帧重复打印相同内容。
    const bool changed = nextState != state_ || nextDirection != direction_;
    state_ = nextState;
    direction_ = nextDirection;
    return changed;
}

MovementState MovementStateMachine::state() const {
    return state_;
}

MovementDirection MovementStateMachine::direction() const {
    return direction_;
}

const char* movementStateName(MovementState state) {
    switch (state) {
        case MovementState::Standing:
            return "Standing";
        case MovementState::Moving:
            return "Moving";
    }
    return "Unknown";
}

const char* movementDirectionName(MovementDirection direction) {
    switch (direction) {
        case MovementDirection::Stopped:
            return "Stopped";
        case MovementDirection::Forward:
            return "Forward";
        case MovementDirection::Backward:
            return "Backward";
    }
    return "Unknown";
}
