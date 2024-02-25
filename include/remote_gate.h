#ifndef HOVERGATE_V2_FW_REMOTE_GATE_H
#define HOVERGATE_V2_FW_REMOTE_GATE_H

//remote gate is used only on master (short) gate

#include "gate_state.h"
#include <stdint.h>
#include <comms.h>

class RemoteGate {
    public:
        RemoteGate(CommsEspNow *comms);
        bool begin();
        void open();
        void close();
        void toggle();
        void stop();
        void reset();
        GateState get_state();
        const char *get_state_str() { return state_2_str(get_state());}
        float get_angle();
        GateError get_error_code();
        const char *get_error_str() { return error_2_str(get_error_code());}
        float get_battery_voltage();
        bool is_connected();
        void loop();
        void send_cmd_raw(uint8_t cmd);

    private:
        CommsEspNow *comms;
        t_msg_esp_now last_msg;
        uint32_t last_msg_time;
        const uint32_t last_msg_timeout = 1000; //ms

};

#endif
