//
// Created by Matej on 20/2/2024.
//

// 

#ifndef HOVERGATE_V2_FW_FW_LATCH_H
#define HOVERGATE_V2_FW_FW_LATCH_H

#include <stdint.h>
#include <Arduino.h>

#define latch_sply_voltage 36.0f //V

#define latch_pwm_pin 35
#define latch_ledc_ch_to_use 0
#define latch_ledc_freq 10000
#define latch_ledc_res 10 //bit
#define latch_ledc_maxval ((1<<latch_ledc_res) - 1)

class Latch {
public:
    Latch();
    void begin();
    void retract();
    void extend();
    void handler();
private:
    const uint32_t latch_retract_time = 800; // ms
    const float latch_retract_voltage = 12; //V
    const float latch_hold_voltage = 6; //V

    uint8_t cmd_state = 0;

    void set_voltage_pwm(float voltage);

};


#endif //HOVERGATE_V2_FW_LATCH_H