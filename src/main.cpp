#include <Arduino.h>
#include "BLDC_driver.h"
#include "latch.h"

BLDC_driver BLDC;
Latch latch;



void setup() {
  pinMode(11, OUTPUT);
  Serial.begin(115200);
  delay(3000);
  Serial.println("Booted HoverGate V2!!!");
  BLDC.begin();
  // BLDC.enable();

  latch.begin();


  Serial.println("BLDC driver inited, entering loop...");

}



void loop() {
  static uint32_t n = 0;
  static uint32_t last_t = millis();
  // put your main code here, to run repeatedly:
  digitalWrite(11, HIGH);
  BLDC.handler();
  digitalWrite(11, LOW);
  latch.handler();
  

  // 1 per sec event
  if(millis() - last_t > 1000){
    last_t = millis();

    if(n==5){
      BLDC.enable();
      BLDC.set_pwm(50);
      latch.retract();
      Serial.println("BLDC set to 50");

    }

    if(n==10){
      BLDC.set_pwm(-50);
      latch.retract();
      Serial.println("BLDC set to -50");

    }
    // if(n==1){
    //   BLDC.set_pwm(100);
    //   Serial.println("BLDC set to 100");

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
    if(n==20){
      BLDC.disable();
      BLDC.reset_encoder();
      latch.extend();
      Serial.println("BLDC dissable");

    }
    


    // Serial.println("phase current: " + String(BLDC.get_current()));
    // Serial.println("angle " + String(BLDC.get_angle()));


    n++;
  }

}

