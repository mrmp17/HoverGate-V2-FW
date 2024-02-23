// ESP-NOW comms
#ifndef HOVERGATE_V2_FW_COMMS_H
#define HOVERGATE_V2_FW_COMMS_H

// short gate is master (has RF, implements MQTT, commands long)


#include "secrets.h"
#include <WiFi.h>
#include <esp_now.h>
#include <comms_msg_structs.h>


class CommsEspNow { 
    public:
        CommsEspNow(const uint8_t peer_mac[6], const char* ssid, const char *pass);
        int begin();
        int send_msg(t_msg_esp_now msg);
        bool is_recv_available();
        t_msg_esp_now get_recv_msg();


    private:
        char _wifi_ssid[32];
        char _wifi_pass[64];
        uint8_t _peer_mac[6];
        const uint32_t _espnow_send_timeout = 20; //ms

};


#endif