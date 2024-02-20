//
// Created by matej on 19/2/2024.
//

#include "BLDC_driver.h"

//why is this below here, not inside the class?
// - access this from other code (eg. from main) might be usefull during debugging
// - BLDCMotor, etc, classes have no default constructors, can't alocate them in class without params
// - hall interrupt handler function pointers cant be passed to register as callbacks if inside class

//simplefoc motor
static BLDCMotor drv_motor = BLDCMotor(drv_polePairs);
//simplefoc driver
static BLDCDriver3PWM drv_driver = BLDCDriver3PWM(drv_pwmA_pin, drv_pwmB_pin, drv_pwmC_pin, drv_enable_pin);
//simplefoc hall sensor
static HallSensor drv_sensor = HallSensor(drv_hallA_pin, drv_hallB_pin, drv_hallC_pin, drv_polePairs);
//simplefoc current sensor
static LowsideCurrentSense drv_current_sense = LowsideCurrentSense(drv_shunt_res, drv_shunt_gain, drv_snsA_pin, drv_snsB_pin, drv_snsC_pin);

// interrupt routine initialization
void drv_int_hall_A(){drv_sensor.handleA();}
void drv_int_hall_B(){drv_sensor.handleB();}
void drv_int_hall_C(){drv_sensor.handleC();}




// constructor
BLDC_driver::BLDC_driver(){
}



void BLDC_driver::begin(){
    //todo: implement hardcoded calibration for halls and current sense to skip alignment
    //todo: set current limits?

    //needed to init current sense adc channels for some reason??? looks like SimpleFOC bug
    analogRead(drv_snsA_pin);
    analogRead(drv_snsB_pin);
    analogRead(drv_snsC_pin);

    //enable simplefoc debug if flag set in .h
    #ifdef SIMPLEFOC_DEBUG_ENABLE
    SimpleFOCDebug::enable();
    #endif

    // initialize hall sensor hardware
    drv_sensor.init();
    //register callbacks for hall interrupts
    drv_sensor.enableInterrupts(drv_int_hall_A, drv_int_hall_B, drv_int_hall_C);

    //link sensor to motor
    drv_motor.linkSensor(&drv_sensor);

    // set power supply voltage param
    drv_driver.voltage_power_supply = drv_power_sply;
    // limit max voltage driver can set
    drv_driver.voltage_limit = drv_driver_volt_limit;

    //init driver
    drv_driver.init();

    //link motor and driver
    drv_motor.linkDriver(&drv_driver);

    //voltage for hall align
    drv_motor.voltage_sensor_align = drv_align_voltage;

    //select FOC modulation
    drv_motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    //link current sense to driver
    drv_current_sense.linkDriver(&drv_driver);

    //set torque and motion controller
    #ifdef drv_torque_control_voltage
    drv_motor.torque_controller = TorqueControlType::voltage;
    #endif
    #ifdef drv_torque_control_phase_current_ampl
    drv_motor.torque_controller = TorqueControlType::dc_current;
    #endif
    #ifdef drv_torque_control_foc_current
    drv_motor.torque_controller = TorqueControlType::foc_current;
    #endif
    
    drv_motor.controller = MotionControlType::torque;

    //init motor
    drv_motor.init();
    //todo: check if successful needed?

    //init current sense
    if (drv_current_sense.init())  Serial.println("Current sense init success!");
    else{
        Serial.println("Current sense init failed!");
        return;
    }

    //link current sense to motor
    drv_motor.linkCurrentSense(&drv_current_sense);

    //align current sense
    drv_current_sense.driverAlign(drv_align_voltage);

    //init FOC
    drv_motor.initFOC();

    //todo: is driver now enabled or not? check!


}

void BLDC_driver::enable(){
    drv_driver.enable();
    BLDC_enabled = true;
}

void BLDC_driver::disable(){
    drv_driver.disable();
    BLDC_enabled = false;
}

bool BLDC_driver::is_enabled(){
    return BLDC_enabled;
}

void BLDC_driver::set_pwm(int16_t pwm){
    if(pwm>1000) pwm = 1000;
    if(pwm<-1000) pwm = -1000;

    #ifdef drv_torque_control_voltage
    drv_motor.move(pwm/1000.0f*drv_max_usr_volt);
    #endif
    #ifdef drv_torque_control_phase_current_ampl
    drv_motor.move(pwm/1000.0f*drv_max_usr_torque_curr);
    #endif
    #ifdef drv_torque_control_foc_current
    rv_motor.move(pwm/1000.0f*drv_max_usr_torque_curr);
    #endif
}

int32_t BLDC_driver::get_encoder(){
    return int((((drv_sensor.getAngle()-encoder_zero_val)/6.28318530718f)*sw_encoder_ticks_per_rev));
}

void BLDC_driver::reset_encoder(){
    encoder_zero_val = drv_sensor.getAngle();
}

float BLDC_driver::get_angle(){
    //substract zero value and convert to degrees
    return ((drv_sensor.getAngle()-encoder_zero_val)/6.28318530718f)*360.0f;
}

void BLDC_driver::handler(){
    drv_motor.loopFOC();
}
