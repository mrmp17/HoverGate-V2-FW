#include <Arduino.h>
#include "BLDC_driver.h"
#include "latch.h"
#include "gate.h"
#include <WiFi.h>
#include <esp_now.h>
#include "secrets.h"
#include "comms.h"
#include "remote_gate.h"

// #define axupin 11

// short gate is master (has RF, implements MQTT, commands long)

#if defined(GATE_SHORT) + defined(GATE_LONG) != 1
#error "GATE_SHORT or GATE_LONG must be defined"
#endif

#define SLAVE_COMMS_INTERVAL 250 // ms

#if defined(GATE_SHORT)
gate_params params {
    .loop_dt = 10, // milliseconds between loops
    .enc_ticks_per_deg = 2.725, // encoder ticks per degree of gate angle
    .angle_open = 90.0, // angle when gate open
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
uint8_t mac_addr[] = mac_gate_long;
CommsEspNow comms(mac_addr, wifi_ssid, wifi_password);
RemoteGate remote_gate(&comms);
#elif defined(GATE_LONG)
gate_params params {
    .loop_dt = 10, // milliseconds between loops
    .enc_ticks_per_deg = 6.1111, // encoder ticks per degree of gate angle
    .angle_open = -90.0, // angle when gate open
    .angle_closed = 0.0, // angle when gate closed
    .a_max = 4,
    .v_max = 12.0,
    .v_min = 4.0,
    .driver_open_dir = 1, // driver pwm sign for open direction. 1 or -1.
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
Latch latch;
uint8_t mac_addr[] = mac_gate_short;
CommsEspNow comms(mac_addr, wifi_ssid, wifi_password);
#endif

BLDC_driver BLDC;
Gate gate(params);

void BLDC_HandlerTask(void *pvParameters){
  while(1){
    BLDC.handler();
    vTaskDelay(1);
  }
}

void gate_HandlerTask(void *pvParameters){
  while(1){
    gate.loop();
    vTaskDelay(10);
  }
}

void commsTask(void *pvParameters){
  while(1){
    #if defined(GATE_SHORT)
    remote_gate.loop();

    // comms handling for long gate
    #elif defined(GATE_LONG)
    // do incoming commands, periodic state report back to master
    static unsigned long last_sent_time = millis();
    if(millis() - last_sent_time > SLAVE_COMMS_INTERVAL){
      uint32_t t = millis();
      digitalWrite(46, HIGH);
      delay(10);
      digitalWrite(46, LOW);
      t_msg_esp_now msg;
      msg.gate_state = static_cast<uint8_t>(gate.get_state());
      msg.error_code = static_cast<uint8_t>(gate.get_error_code());
      msg.gate_angle = gate.get_angle();
      msg.bat_volt = 0; //todo: implement
      msg.action_cmd = 0; // not in use. set to 0
      msg.isShort = false;
      comms.send_msg(msg);
      //todo: need to do sth if this fails?
      // Serial.println("took " + String(millis()-t) + "ms to send msg.");
      //todo: what if delivery is not ok?
      last_sent_time = millis();
    }

    // do incoming commands
    if(comms.is_recv_available()){
      t_msg_esp_now msg = comms.get_recv_msg();
      switch(msg.action_cmd) {
      case 0:
          gate.open();
          break;
      case 1:
          gate.close();
          break;
      case 2:
          gate.reset();
          break;
      case 3:
          gate.stop();
          break;
      case 4:
          gate.toggle();
          break;
      //for testing
      case 5:
        Serial.println("Received action_cmd 5");
        break;
      default:
          // Handle invalid action_cmd values
          break;
}
    }
    #endif
    vTaskDelay(10);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(46, OUTPUT);
  delay(3000);
  Serial.println("Booted HoverGate V2!!!");

  #if defined(GATE_SHORT)
  {
    Serial.println("This is: SHORT GATE / MASTER");
    BLDC.set_hall_align_param(4.188790f, -1);
    remote_gate.begin();
  }
  #elif defined(GATE_LONG)
  {
    Serial.println("This is: LONG GATE / SLAVE");
    BLDC.set_hall_align_param(4.188791f, 1);
    gate.set_latch(&latch);
    comms.begin();
  }
  #endif

  gate.set_driver(&BLDC);
  gate.begin();

  xTaskCreate(
    commsTask,  /* Task function. */
    "commsTask",  /* String with name of task. */
    2048,  /* Stack size in bytes. */
    NULL,  /* Parameter passed as input of the task */
    1,  /* Priority of the task. */
    NULL);  /* Task handle. */
  Serial.println("Started commsTask");


  xTaskCreatePinnedToCore(
    BLDC_HandlerTask,  /* Task function. */
    "BLDC_HandlerTask",  /* String with name of task. */
    4095,  /* Stack size in bytes. */
    NULL,  /* Parameter passed as input of the task */
    20,  /* Priority of the task. */
    NULL,  /* Task handle. */
    1); // core 1
  Serial.println("Started BLDC_HandlerTask");

  xTaskCreate(
    gate_HandlerTask,  /* Task function. */
    "gate_HandlerTask",  /* String with name of task. */
    2048,  /* Stack size in bytes. */
    NULL,  /* Parameter passed as input of the task */
    1,  /* Priority of the task. */
    NULL);  /* Task handle. */
  Serial.println("gate_HandlerTask");

  
  

  // //needed only for long gate
  // #ifndef GATE_SHORT
  // xTaskCreate(
  //   longGatePeriodicSendMsgTask,  /* Task function. */
  //   "longGatePeriodicSendMsgTask",  /* String with name of task. */
  //   2048,  /* Stack size in bytes. */
  //   NULL,  /* Parameter passed as input of the task */
  //   5,  /* Priority of the task. */
  //   NULL);  /* Task handle. */
  // Serial.println("Started longGatePeriodicSendMsgTask");
  // #endif
  
  Serial.println("Entering loop...");


}

void loop() {
  static uint32_t n = 0;
  static uint32_t last_t = millis();

  // 1 per sec event
  if(millis() - last_t > 1000){
    last_t = millis();

    // if(n==2){
    //     BLDC.enable();
    //     BLDC.set_pwm(60);
    //     delay(3000);
    //     BLDC.set_pwm(0);
    //     BLDC.disable();
    //     // Serial.println("gate open");
    // }
    // if(n==3){
    //   n = 2;
    // }
    #if defined(GATE_SHORT)
    if(n==5){
      gate.open();
      Serial.println("gate open");
    }

    if(n==7){
      remote_gate.open();
      Serial.println("remote gate open");
    }

    if(n==25){
      remote_gate.close();
      Serial.println("remote gate close");
    }

    if(n==35){
      gate.close();
      Serial.println("gate close");
    }
    #endif

    // #ifdef GATE_SHORT
    // if(n%4 == 0){
    //   remote_gate.send_cmd_raw(5);
    // }
    // #endif

    if(n==30){
      gate.close();
      Serial.println("gate close");
    }
    
    // if(n==2){
    //   BLDC.enable();
    //   BLDC.set_pwm(0);
    //   Serial.println("BLDC set to 0");
    // }

    // if(n==5){
    //   BLDC.enable();
    //   BLDC.set_pwm(50);
    //   Serial.println("BLDC set to 50");

    // }
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
    // if(n==20){
    //   BLDC.disable();
    //   BLDC.set_pwm(0);
    //   BLDC.reset_encoder();
    //   Serial.println("BLDC dissable");

    // }
    


    // Serial.println("phase current: " + String(BLDC.get_current()));
    // Serial.println("angle " + String(BLDC.get_angle()));
    #if defined(GATE_SHORT)
    Serial.printf("remote connected: %d\n", remote_gate.is_connected());
    Serial.printf("gate state: %s\n", gate.get_state_str());
    Serial.printf("gate angle: %f\n", gate.get_angle());
    Serial.printf("error code: %s\n", gate.get_error_str());
    Serial.printf("remote gate state: %s\n", remote_gate.get_state_str());
    Serial.printf("remote error code: %s\n", remote_gate.get_error_str());
    Serial.printf("remote gate angle: %f\n", remote_gate.get_angle());

    #elif defined(GATE_LONG)
    Serial.printf("gate state: %s\n", gate.get_state_str());
    Serial.printf("gate angle: %f\n", gate.get_angle());
    Serial.printf("error code: %s\n", gate.get_error_str());
    #endif

    n++;
  }

}

