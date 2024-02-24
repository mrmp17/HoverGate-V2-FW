#ifndef GATE_STATE_H
#define GATE_STATE_H

enum class GateState {
        closed = 0,
        opening = 1,
        open = 2,
        closing = 3,
        error = 4,
        not_connected = 5
    };

enum class GateCmd {
    open = 0,
    close = 1,
    reset = 2,
    stop = 3,
    toggle = 4
};

enum class ErrorCode {
    no_error = 0,
    stopped_before_expected = 1,
    not_stopped_expected = 2,
    stopped_manually = 3,
    not_connected = 4
};

inline const char * state_2_str(GateState state) {
    switch (state) {
        case GateState::closed: return "closed";
        case GateState::opening: return "opening";
        case GateState::open: return "open";
        case GateState::closing: return "closing";
        case GateState::error: return "error";
        case GateState::not_connected: return "not connected";
    }
    return "unknown";
}

inline const char * error_2_str(ErrorCode error) {
    switch (error) {
        case ErrorCode::no_error: return "no error";
        case ErrorCode::stopped_before_expected: return "stopped before expected";
        case ErrorCode::not_stopped_expected: return "not stopped expected";
        case ErrorCode::stopped_manually: return "stopped manually";
        case ErrorCode::not_connected: return "not connected";
    }
    return "unknown";
}

#endif