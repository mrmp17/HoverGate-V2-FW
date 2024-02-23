#ifndef HOVERGATE_V2_FW_REMOTE_GATE_H
#define HOVERGATE_V2_FW_REMOTE_GATE_H

//remote gate is used only on master (short) gate

/*
* Error codes:
* 0 - no error
* 1 - gate stopped before expected position
* 2 - gate did not stop at expected position
* 3 - stopped manually
* 4 - connection lost (only on remote gate)
*/


#include <stdint.h>
#include <comms.h>

#endif

class RemoteGate {
    public:
        enum class GateState {
            closed = 0,
            opening = 1,
            open = 2,
            closing = 3,
            error = 4,
            not_connected = 5
        };
        RemoteGate(CommsEspNow *comms);
        void begin();
        void open();
        void close();
        void toggle();
        void stop();
        void reset();
        GateState get_state();
        float get_angle();
        uint8_t get_error_code();
        float get_battery_voltage();
        bool is_connected();
        void loop();

    private:
        CommsEspNow *comms;
        t_msg_esp_now last_msg;
        uint32_t last_msg_time;
        const uint32_t last_msg_timeout = 1000; //ms

};