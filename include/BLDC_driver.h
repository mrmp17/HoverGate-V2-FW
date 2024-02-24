//
// Created by matej on 19/2/2024.
//

#ifndef HOVERGATE_V2_FW_BLDC_DRIVER_H
#define HOVERGATE_V2_FW_BLDC_DRIVER_H


#include "driver.h"
#include <SimpleFOC.h>

#define drv_power_sply 36.0f

#define drv_polePairs 15
#define drv_pwmA_pin 12
#define drv_pwmB_pin 13
#define drv_pwmC_pin 14
#define drv_enable_pin 41
#define drv_snsA_pin 4
#define drv_snsB_pin 6
#define drv_snsC_pin 5
#define drv_hallA_pin 38
#define drv_hallB_pin 37
#define drv_hallC_pin 36
#define drv_cal_pin 40
#define drv_shunt_res 0.003f
#define drv_shunt_gain (-20.0f)
#define drv_driver_volt_limit 8.0f
#define drv_align_voltage 2.0f

//set_pwm maps -1000 to 1000 maps to +- these values (depending on voltage or phase current torque control)
#define drv_max_usr_volt 24.0f
#define drv_max_usr_torque_curr 10.0f

//output of get_encoder is in ticks per rev (90 for hub motor) (from legacy implementation)
#define sw_encoder_ticks_per_rev 90

// hardcoded hall offset and direction (used only if #define drv_skip_hall_align)
#define drv_hall_offset 4.188790f
#define drv_hall_direction CCW

// select torque controller
#define drv_torque_control_voltage
// #define drv_torque_control_phase_current_ampl
// #define drv_torque_control_foc_current

// select debug
#define SIMPLEFOC_DEBUG_ENABLE

//skip current sense alignment (if pins and gains set correctly, not needed)
#define drv_skip_current_sense_align

//skip hall sensor alignment (if offset and direction known, not needed)
#define drv_skip_hall_align





class BLDC_driver : public Driver {
public:
    BLDC_driver();
    void begin() override;
    void enable() override;
    bool is_enabled() override;
    void disable() override;
    void set_pwm(int16_t pwm) override;  //+- 1000 //TODO: map to 0-2k timer values
    void handler() override;
    // int16_t get_pwm() override;
    int32_t get_encoder() override;
    float get_angle() override;
    void reset_encoder() override;
    float get_current() override;
    // void ramp_pwm(int16_t pwm_to, uint32_t time_ms) override;
    // bool is_ramp_active() override;

    // void interrupt_handler(); //call this at 10kHz
    // void auto_pwm_handler();  //call this inside interrupt_handler

    //uint8_t forceStep = 1;
    // uint8_t _loc = 0;


private:

    bool BLDC_enabled = false;
    bool BLDC_direction = false;
    uint16_t BLDC_pwm_set_value = 0; //this should be 0-2k - direct pwm value
    int16_t BLDC_user_pwm = 0;  //this is value from set_pwm call (-1000 to 1000)
    float encoder_zero_val = 0;





    // double ramp_k = 0.0;
    // double ramp_n = 0.0;
    // uint32_t ramp_cnt = 0;
    // bool ramp_active = false;
    // uint32_t ramp_end_cnt = 0;


};


#endif //HOVERGATE_V2_FW_BLDC_DRIVER_H