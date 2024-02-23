// ESP-NOW comms
#ifndef HOVERGATE_V2_FW_COMMS_MSG_STRUCTS_H
#define HOVERGATE_V2_FW_COMMS_MSG_STRUCTS_H

// short gate is master (has RF, implements MQTT, commands long)

#include <stdint.h>

//typedef for ESP-NOW comms messages
typedef struct MsgEspNow{
    uint8_t gate_state;
    uint8_t error_code;
    float gate_angle;
    float bat_volt;
    uint8_t action_cmd;
    bool isShort;
} t_msg_esp_now;


#endif //HOVERGATE_V2_FW_COMMS_MSG_STRUCTS_H