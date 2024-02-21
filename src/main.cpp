#include <Arduino.h>
#include "BLDC_driver.h"
#include "latch.h"
#include "gate.h"

#define axupin 11

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

BLDC_driver BLDC;
Gate gate(params);


void BLDC_HandlerTask(void *pvParameters){
  while(1){
    digitalWrite(axupin, HIGH);
    BLDC.handler();
    digitalWrite(axupin, LOW);
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

  pinMode(axupin, OUTPUT);

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
  // put your main code here, to run repeatedly:
  // digitalWrite(axupin, HIGH);
  // BLDC.handler();
  // digitalWrite(axupin, LOW);

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

