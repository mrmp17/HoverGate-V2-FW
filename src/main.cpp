#include <Arduino.h>
#include "BLDC_driver.h"
#include "latch.h"
#include "gate.h"
#include <WiFi.h>
#include <esp_now.h>
#include "secrets.h"

// #define axupin 11

// short gate is master (has RF, implements MQTT, commands long)

// select gate. define GATE_SHORT for short gate, comment out for long gate
#define GATE_SHORT // comment if compiling for long gate wing

#ifdef GATE_SHORT
gate_params params {
    .loop_dt = 10, // milliseconds between loops
    .enc_ticks_per_deg = 2.725, // encoder ticks per degree of gate angle
    .angle_open = -90.0, // angle when gate open
    .angle_closed = 0.0, // angle when gate closed
    .a_max = 6.0,
    .v_max = 16.0,
    .v_min = 6.0,
    .driver_open_dir = -1, // driver pwm sign for open direction. 1 or -1.
    .max_pwm = 150, // max driver pwm
    .pid_kp = 30,
    .pid_ki = 2,
    .pid_slow_kp = 10,
    .pid_slow_ki = 0,
    .pid_imax = 300,
    .vel_update_tick_num = 5,
    .zero_vel_timeout = 2000,
    .move_uncert_before = 10.0, // degrees before target when velocity is reduced
    .move_uncert_after = 30.0, // degrees after target when velocity still set
    .max_angle_follow_error = 10.0, // max error when gate stopped is detected
    .hold_open_offset = -5.
};
#else
// Long gate
gate_params params {
    .loop_dt = 10, // milliseconds between loops
    .enc_ticks_per_deg = 6.1111, // encoder ticks per degree of gate angle
    .angle_open = 90.0, // angle when gate open
    .angle_closed = 0.0, // angle when gate closed
    .a_max = 4,
    .v_max = 12.0,
    .v_min = 4.0,
    .driver_open_dir = -1, // driver pwm sign for open direction. 1 or -1.
    .max_pwm = 150, // max driver pwm
    .pid_kp = 30,
    .pid_ki = 2,
    .pid_slow_kp = 10,
    .pid_slow_ki = 0,
    .pid_imax = 300,
    .vel_update_tick_num = 5,
    .zero_vel_timeout = 2000,
    .move_uncert_before = 10.0, // degrees before target when velocity is reduced
    .move_uncert_after = 30.0, // degrees after target when velocity still set
    .max_angle_follow_error = 10.0, // max error when gate stopped is detected
    .hold_open_offset = 6.
};
#endif

#ifndef GATE_SHORT
Latch latch;
#endif

// typedefs for ESP-NOW messages
typedef struct MsgShortToLong {
    uint8_t action_cmd;
} t_msg_short_to_long;

typedef struct MsgLongToShort {
    uint8_t gate_state;
    uint8_t error_code;
    float gate_angle;
    float bat_volt;
} t_msg_long_to_short;

//global variables for ESP-NOW comms messages
t_msg_short_to_long msg_short_to_long;
t_msg_long_to_short msg_long_to_short;
uint32_t msg_num_recvd = 0; //to check for missed messages when processing. increments in callback, decrements at processing
uint32_t msg_deliver_ok = 0; //flag to check if message was sent and delivered

#ifdef GATE_SHORT
// short gate sends to long gate mac
uint8_t send_to_mac_addr[] = mac_gate_long;
#endif
#ifndef GATE_SHORT
// long gate sends to short gate mac
uint8_t send_to_mac_addr[] = mac_gate_short;
#endif




BLDC_driver BLDC;
Gate gate(params);

// ESP-NOW recv callback 
void onEspnowRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
  //short gate is receiving from long
  #ifdef GATE_SHORT
  memcpy(&msg_long_to_short, incomingData, sizeof(msg_long_to_short));
  #endif

  //long gate is receiving from short
  #ifndef GATE_SHORT
  memcpy(&msg_short_to_long, incomingData, sizeof(msg_short_to_long));
  #endif

  msg_num_recvd++;
}

// ESP-NOW sent callback
void onEspnowSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  if(status == ESP_NOW_SEND_SUCCESS){
    msg_deliver_ok = 1;
  }
  else{
    msg_deliver_ok = 0;
  }
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "ESP-NOW packed delivered" : "ESP-NOW delivery fail");
}



void BLDC_HandlerTask(void *pvParameters){
  while(1){
    BLDC.handler();
    vTaskDelay(1);
  }
}

void gate_HandlerTask(void *pvParameters){
  while(1){
    // gate.loop();
    vTaskDelay(10);
  }
}


void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("Booted HoverGate V2!!!");

  //set wifi mode
  WiFi.mode(WIFI_STA);

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
    return;
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
      return;
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
  memcpy(peerInfo.peer_addr, send_to_mac_addr, 6); // Ensure receiver_mac is defined
  peerInfo.channel = 0;  // same as WiFi channel (or default if not on wifi)
  peerInfo.encrypt = false;

  // add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
  }
  else{
    Serial.println("Peer added");
  }


  



  // gate.set_driver(&BLDC);
  #ifndef GATE_SHORT
  gate.set_latch(&latch);
  #endif

  // gate.begin();
  BLDC.begin();

  xTaskCreatePinnedToCore(
    BLDC_HandlerTask,  /* Task function. */
    "BLDC_HandlerTask",  /* String with name of task. */
    4095,  /* Stack size in bytes. */
    NULL,  /* Parameter passed as input of the task */
    20,  /* Priority of the task. */
    NULL,  /* Task handle. */
    1); // core 1
  Serial.println("Started BLDC handler task");

  xTaskCreate(
    gate_HandlerTask,  /* Task function. */
    "gate_HandlerTask",  /* String with name of task. */
    2048,  /* Stack size in bytes. */
    NULL,  /* Parameter passed as input of the task */
    5,  /* Priority of the task. */
    NULL);  /* Task handle. */
  Serial.println("Started gate handler task");

  
  Serial.println("Entering loop...");


}



void loop() {
  static uint32_t n = 0;
  static uint32_t last_t = millis();

  // 1 per sec event
  if(millis() - last_t > 1000){
    last_t = millis();

    // if(n==5){
    //   gate.open();
    //   Serial.println("gate open");

    // }

    // if(n==30){
    //   gate.close();
    //   Serial.println("gate close");

    // }
    if(n==2){
      BLDC.enable();
      BLDC.set_pwm(0);
      Serial.println("BLDC set to 0");
    }

    if(n==5){
      BLDC.enable();
      BLDC.set_pwm(50);
      Serial.println("BLDC set to 50");

    }
    // if(n==3){
    //   BLDC.set_pwm(0);
    //   Serial.println("BLDC set to 0");

    // }
    // if(n==5){
    //   BLDC.set_pwm(-50);
    //   Serial.println("BLDC set to -50");

    // }
    // if(n==8){
    //   BLDC.disable();
    //   Serial.println("BLDC disabled");
    //   BLDC.set_pwm(0);
    // }
    // if(n==10){
    //   BLDC.enable();
    //   BLDC.set_pwm(50);
    //   Serial.println("BLDC enable set to 50");
    // }
    // if(n==15){
    //   BLDC.set_pwm(0);
    //   Serial.println("BLDC set to 0");

    // }
    if(n==20){
      BLDC.disable();
      BLDC.set_pwm(0);
      BLDC.reset_encoder();
      Serial.println("BLDC dissable");

    }
    


    Serial.println("phase current: " + String(BLDC.get_current()));
    // Serial.println("angle " + String(BLDC.get_angle()));


    n++;
  }

}

