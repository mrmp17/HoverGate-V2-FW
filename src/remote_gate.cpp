#include "remote_gate.h"


RemoteGate::RemoteGate(CommsEspNow *comms_obj) {
    comms = comms_obj;
}

void RemoteGate::begin() {
    //begin comms
    switch(comms->begin()){
        case 0:
            Serial.println("Comms begin success");
        break;
        case 1:
            Serial.println("Comms begin ok, but no WiFi connection");
        break;
        case -1:
            Serial.println("Comms begin fail. Stopping here.");
            while(1){
                vTaskDelay(100);
            };
        break;

        default:
        break;
    }
    //wait for first status packet from long gate / slave
    Serial.println("Waiting for remote gate connection...");
    while(!comms->is_recv_available()){
        Serial.print("-");
        vTaskDelay(500);
    }
    t_msg_esp_now firstMsg = comms->get_recv_msg();
    if(firstMsg.isShort){
        //no bueno, we are short gate
        Serial.println("Error: received message with isShort flag set - invalid! Stopping here");
        while(1){
            vTaskDelay(100);
        }
    }
    Serial.println("Remote gate connected!");

}

//gate cmds:
//0 - open
//1 - close
//2 - reset
//3 - stop
//4 - toggle

void RemoteGate::open() {
    t_msg_esp_now msg;
    msg.action_cmd = 0; //0 is open
    comms->send_msg(msg);
}

void RemoteGate::close() {
    t_msg_esp_now msg;
    msg.action_cmd = 1; //1 is close
    comms->send_msg(msg);
}

void RemoteGate::reset() {
    t_msg_esp_now msg;
    msg.action_cmd = 2; //2 is reset
    comms->send_msg(msg);
}

void RemoteGate::stop() {
    t_msg_esp_now msg;
    msg.action_cmd = 3; //3 is stop
    comms->send_msg(msg);
}

void RemoteGate::toggle() {
    t_msg_esp_now msg;
    msg.action_cmd = 4; //3 is toggle
    comms->send_msg(msg);
}

bool RemoteGate::is_connected() {
    return millis()-last_msg_time < last_msg_timeout;
}

RemoteGate::GateState RemoteGate::get_state() {
    if(!is_connected()){
        return GateState::not_connected;
    }
    return (GateState)last_msg.gate_state;
}

float RemoteGate::get_battery_voltage() {
    if(!is_connected()){
        Serial.println("Error: remote gate not connected! Returning last known voltage.");
    }
    return last_msg.bat_volt;
}

float RemoteGate::get_angle() {
    if(!is_connected()){
        Serial.println("Error: remote gate not connected! Returning last known angle.");
    }
    return last_msg.gate_angle;
}

uint8_t RemoteGate::get_error_code() {
    if(is_connected()){
        return last_msg.error_code;
    }
    return 4; //not connected

}

//call this in a loop/task periodically
void RemoteGate::loop() {
    //checks for periodic messages from slave/long, fills in last_msg_time and last_msg
    if(comms->is_recv_available()){
        last_msg = comms->get_recv_msg();
        last_msg_time = millis();
    }
}








