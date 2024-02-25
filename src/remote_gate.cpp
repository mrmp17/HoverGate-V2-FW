#include "remote_gate.h"


RemoteGate::RemoteGate(CommsEspNow *comms_obj) {
    comms = comms_obj;
}

bool RemoteGate::begin() {
    //begin comms
    pinMode(46, OUTPUT);
    switch(comms->begin()){
        case 0:
            Serial.println("Comms begin success");
            break;
        case 1:
            Serial.println("Comms begin ok, but no WiFi connection");
            break;
        case -1:
            Serial.println("Comms begin fail. Stopping here.");
            return false;
        default:
            break;
    }
    //wait for first status packet from long gate / slave
    Serial.println("Waiting for remote gate connection...");
    while(!comms->is_recv_available()){
        vTaskDelay(100);
    }
    t_msg_esp_now firstMsg = comms->get_recv_msg();
    if(firstMsg.isShort){
        //no bueno, we are short gate
        Serial.println("Error: received message with isShort flag set - invalid! Stopping here");
        return false;
    }
    Serial.println("Remote gate connected!");
    return true;
}

void RemoteGate::open() {
    t_msg_esp_now msg;
    msg.action_cmd = static_cast<uint8_t>(GateCmd::open);
    comms->send_msg(msg);
}

void RemoteGate::close() {
    t_msg_esp_now msg;
    msg.action_cmd = static_cast<uint8_t>(GateCmd::close);
    comms->send_msg(msg);
}

void RemoteGate::reset() {
    t_msg_esp_now msg;
    msg.action_cmd = static_cast<uint8_t>(GateCmd::reset);
    comms->send_msg(msg);
}

void RemoteGate::stop() {
    t_msg_esp_now msg;
    msg.action_cmd = static_cast<uint8_t>(GateCmd::stop);
    comms->send_msg(msg);
}

void RemoteGate::toggle() {
    t_msg_esp_now msg;
    msg.action_cmd = static_cast<uint8_t>(GateCmd::toggle);
    comms->send_msg(msg);
}

//send raw command
void RemoteGate::send_cmd_raw(uint8_t cmd) {
    t_msg_esp_now msg;
    msg.action_cmd = cmd;
    comms->send_msg(msg);
}

bool RemoteGate::is_connected() {
    return millis() - last_msg_time < last_msg_timeout;
}

GateState RemoteGate::get_state() {
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

GateError RemoteGate::get_error_code() {
    if(is_connected()){
        return static_cast<GateError>(last_msg.error_code);
    }
    return  GateError::not_connected;

}

//call this in a loop/task periodically
void RemoteGate::loop() {
    //checks for periodic messages from slave/long, fills in last_msg_time and last_msg
    if(comms->is_recv_available()){
        last_msg = comms->get_recv_msg();
        last_msg_time = millis();
        digitalWrite(46, HIGH);
        delay(10);
        digitalWrite(46, LOW);
    }
}








