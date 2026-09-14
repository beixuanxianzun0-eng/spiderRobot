#include "movement_state_machine.h"

bool MovementStateMachine::update(
    bool forwardPressed,
    bool backwardPressed,
    bool leftPressed,
    bool rightPressed
) {
    const int forwardAxis
        = static_cast<int>(forwardPressed) - static_cast<int>(backwardPressed);
    const int leftAxis
        = static_cast<int>(leftPressed) - static_cast<int>(rightPressed);

    MovementState nextState = MovementState::Moving;
    MovementDirection nextDirection = MovementDirection::Stopped;
    if (forwardAxis > 0 && leftAxis > 0) {
        nextDirection = MovementDirection::ForwardLeft;
    } else if (forwardAxis > 0 && leftAxis < 0) {
        nextDirection = MovementDirection::ForwardRight;
    } else if (forwardAxis < 0 && leftAxis > 0) {
        nextDirection = MovementDirection::BackwardLeft;
    } else if (forwardAxis < 0 && leftAxis < 0) {
        nextDirection = MovementDirection::BackwardRight;
    } else if (forwardAxis > 0) {
        nextDirection = MovementDirection::Forward;
    } else if (forwardAxis < 0) {
        nextDirection = MovementDirection::Backward;
    } else if (leftAxis > 0) {
        nextDirection = MovementDirection::Left;
    } else if (leftAxis < 0) {
        nextDirection = MovementDirection::Right;
    } else {
        nextState = MovementState::Standing;
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
        case MovementDirection::Left:
            return "Left";
        case MovementDirection::Right:
            return "Right";
        case MovementDirection::ForwardLeft:
            return "ForwardLeft";
        case MovementDirection::ForwardRight:
            return "ForwardRight";
        case MovementDirection::BackwardLeft:
            return "BackwardLeft";
        case MovementDirection::BackwardRight:
            return "BackwardRight";
    }
    return "Unknown";
}
