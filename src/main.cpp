#include <Arduino.h>
#include "BLDC_driver.h"

BLDC_driver BLDC;


void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("Booted HoverGate V2!!!");
  BLDC.begin();
  BLDC.enable();
  Serial.println("BLDC driver inited, entering loop...");

}

void loop() {
  static uint32_t n = 0;
  static uint32_t last_t = millis();
  // put your main code here, to run repeatedly:
  BLDC.handler();

  // 1 per sec event
  if(millis() - last_t > 1000){
    last_t = millis();

    if(n==0){
      BLDC.set_pwm(50);
      Serial.println("BLDC set to 50");

    }
    if(n==5){
      BLDC.set_pwm(-50);
      Serial.println("BLDC set to 50");

    }
    if(n==10){
      BLDC.disable();
      Serial.println("BLDC disabled");
    }

    n++;
  }

}

