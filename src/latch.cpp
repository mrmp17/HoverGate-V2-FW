//
// Created by Matej on 20/2/2024.
//

#include "latch.h"

Latch::Latch() {

}

// Warning: need to call begin (different from before porting to HoverGate V2 hardware)
void Latch::begin() {
    ledcSetup(latch_ledc_ch_to_use, latch_ledc_freq, latch_ledc_res); // Channel, Frequency, Resolution
    ledcAttachPin(latch_pwm_pin, latch_ledc_ch_to_use); // Attach the channel to the GPIO pin
    set_voltage_pwm(0.0); // set to zero (off)

}

void Latch::retract() {
    cmd_state = 1;
}

void Latch::extend() {
    cmd_state = 0;
}

void Latch::handler() {
    static uint32_t retract_start_time = 0;
    static uint8_t loopCtrl = 0;
    switch(loopCtrl){
        case 0:
            //latch current zero 
            set_voltage_pwm(0.0);
            if(cmd_state == 1) loopCtrl = 1;
            break;
        case 1:
            //set latch current to full
            set_voltage_pwm(latch_retract_voltage);
            retract_start_time = millis();
            if(cmd_state == 0) loopCtrl = 0;
            loopCtrl = 2;
            break;
        case 2:
            if(millis() - retract_start_time >= latch_retract_time){
                //set current to hold setting
                set_voltage_pwm(latch_hold_voltage);
                loopCtrl = 3;
            }
            if(cmd_state == 0) loopCtrl = 0;
            break;
        case 3:
            if(cmd_state == 0) loopCtrl = 0;
            break;
    }
}

void Latch::set_voltage_pwm(float voltage) {
    if(voltage == 0.0){
        ledcWrite(latch_ledc_ch_to_use, 0);
        return;
    }
    uint32_t pwm_val = uint32_t((voltage/latch_sply_voltage) * latch_ledc_maxval);
    if(pwm_val > latch_ledc_maxval) pwm_val = latch_ledc_maxval; //cap at max
    ledcWrite(latch_ledc_ch_to_use, pwm_val);
}