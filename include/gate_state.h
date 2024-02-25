#ifndef GATE_STATE_H
#define GATE_STATE_H

enum class GateState {
        closed = 0,
        opening = 1,
        open = 2,
        closing = 3,
        error = 4,
        not_connected = 5,
        resetting = 6
    };

enum class GateCmd {
    open = 0,
    close = 1,
    reset = 2,
    stop = 3,
    toggle = 4
};

enum class GateError {
    gate_ok = 0,
    stopped_before_expected = 1,
    not_stopped_expected = 2,
    stopped_manually = 3,
    not_connected = 4,
    short_gate_comm_dead = 5,
    long_gate_comm_dead = 6,
    short_gate_error = 7,
    long_gate_error = 8
};

inline const char * state_2_str(GateState state) {
    switch (state) {
        case GateState::closed: return "closed";
        case GateState::opening: return "opening";
        case GateState::open: return "open";
        case GateState::closing: return "closing";
        case GateState::error: return "error";
        case GateState::not_connected: return "not connected";
        case GateState::resetting: return "resetting";
    }
    return "unknown";
}

inline const char * error_2_str(GateError error) {
    switch (error) {
        case GateError::gate_ok: return "no error";
        case GateError::stopped_before_expected: return "stopped before expected";
        case GateError::not_stopped_expected: return "not stopped expected";
        case GateError::stopped_manually: return "stopped manually";
        case GateError::not_connected: return "not connected";
        case GateError::short_gate_comm_dead: return "short gate comm dead";
        case GateError::long_gate_comm_dead: return "long gate comm dead";
        case GateError::short_gate_error: return "short gate error";
        case GateError::long_gate_error: return "long gate error";
    }
    return "unknown";
}

#endif