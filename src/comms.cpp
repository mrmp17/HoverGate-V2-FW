#include "comms.h"

// global variables for ESP-NOW comms messages
// global becaus callback register doesnt support bind
static t_msg_esp_now msg_esp_now_recv;
static uint32_t msg_num_recvd = 0; //to check for missed messages when processing. increments in callback, decrements at processing
static int msg_deliver_status = -1; //flag to check if message was sent and delivered

// ESP-NOW callbacks
// global becaus callback register doesnt support bind

// ESP-NOW receive callback
void onEspnowRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
  memcpy(&msg_esp_now_recv, incomingData, sizeof(msg_esp_now_recv));
  msg_num_recvd++;
}

// ESP-NOW sent callback
void onEspnowSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  if(status == ESP_NOW_SEND_SUCCESS){
    msg_deliver_status = 1;
  }
  else{
    msg_deliver_status = 0;
  }
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "ESP-NOW packed delivered" : "ESP-NOW delivery fail");
}
// #################



CommsEspNow::CommsEspNow(const uint8_t peer_mac[6], const char* ssid, const char *pass){
    memcpy(_peer_mac, peer_mac, 6);
    strcpy(_wifi_ssid, ssid);
    strcpy(_wifi_pass, pass);
}

// connects to wifi, sets up ESP-NOW and adds peer
// returns 0 on success, -1 on critical error, 1 on wifi connection fail
int CommsEspNow::begin(){
    int success = 0;
    Serial.println("Connecting to Wi-Fi...");
    unsigned long startConnect = millis();
    // connecting to WiFi so ESP-NOW is on the same channel
    WiFi.begin("ssid", "password");
    // Wait until the connection has been established. 10s timeout
    while (WiFi.status() != WL_CONNECTED && millis() - startConnect < 10000) {
        delay(500);
        Serial.print(".");
    }
    if(WiFi.status() != WL_CONNECTED){
        Serial.println("Failed to connect to Wi-Fi. ESP-NOW will use default channel. Will not retry WiFi connection.");
        success = 1;
    }
    else{
        //connected, print IP and channel
        Serial.println("Connected to Wi-Fi");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Wi-Fi Channel: ");
        Serial.println(WiFi.channel());
    }

    //init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        success = -1;
        return success;
        //no bueno without ESP-NOW
    }
    else{
        Serial.println("ESP-NOW init OK!");
    }

    //register ESP-NOW callbacks
    esp_now_register_recv_cb(onEspnowRecv);
    esp_now_register_send_cb(onEspnowSent);

    // add ESP-NOW peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo)); //all zeros
    memcpy(peerInfo.peer_addr, _peer_mac, 6); // Ensure receiver_mac is defined
    peerInfo.channel = 0;  // same as WiFi channel (or default if not on wifi)
    peerInfo.encrypt = false;

    // add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        success = -1;
        return success;
        //no bueno without peer
    }
    else{
        Serial.println("Peer added");
    }
    return success;
}

int CommsEspNow::send_msg(t_msg_esp_now msg){
    //send message
    esp_now_send(NULL, (uint8_t *) &msg, sizeof(msg));
    //wait for delivery
    unsigned long startWait = millis();
    while(msg_deliver_status == -1 && millis() - startWait < _espnow_send_timeout){
        vTaskDelay(1);
    }
    return msg_deliver_status;
}

bool CommsEspNow::is_recv_available(){
    return msg_num_recvd > 0;
}

t_msg_esp_now CommsEspNow::get_recv_msg(){
    msg_num_recvd = 0;
    return msg_esp_now_recv;
}