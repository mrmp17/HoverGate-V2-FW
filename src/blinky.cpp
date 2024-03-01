#include "blinky.h"

Blinky::Blinky(uint8_t pin){
    this->pin = pin;
}


void Blinky::begin(){
    pinMode(pin, OUTPUT);
}
void Blinky::set_blink_normal(){
    blink_on_now = blink_normal_on;
    blink_off_now = blink_normal_off;
    solid_on = false;
    solid_off = false;
}
void Blinky::set_blink_error(){
    blink_on_now = blink_error_on;
    blink_off_now = blink_error_off;
    solid_on = false;
    solid_off = false;
}
void Blinky::set_on(){
    digitalWrite(pin, HIGH); //directly set pin to work even without handler
    solid_on = true;
}
void Blinky::set_off(){
    digitalWrite(pin, LOW); //directly set pin to work even without handler
    solid_off = true;
}
void Blinky::handler(){
    static uint32_t timevar = millis();
    static uint32_t loopCtrl = 0;

    // led is off
    if(solid_off){
        digitalWrite(pin, LOW);
        loopCtrl = 0;
    }

    //led is solid on
    else if(solid_on){
        digitalWrite(pin, HIGH);
        loopCtrl = 0;
    }

    //led is blinking (timing depends on mode set by set_blink_normal or set_blink_error)
    else{
        switch(loopCtrl){
            case 0:
                digitalWrite(pin, HIGH);
                timevar = millis();
                loopCtrl = 1;
                break;
            case 1:
                if(millis() - timevar > blink_on_now){
                    digitalWrite(pin, LOW);
                    timevar = millis();
                    loopCtrl = 2;
                }
                break;
            case 2:
                if(millis() - timevar > blink_off_now){
                    loopCtrl = 0;
                }
                break;
        }
    }
    
}