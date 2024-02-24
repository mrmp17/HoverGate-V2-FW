//
// Created by Nejc on 25/12/2019.
//

#ifndef HOVERGATE_V2_FW_GATE_H
#define HOVERGATE_V2_FW_GATE_H

#include "gate_state.h"
#include "driver.h"
#include "pid.h"
#include "math.h"
#include "latch.h"


struct gate_params {
    uint16_t loop_dt; // milliseconds between loops
    double enc_ticks_per_deg; // encoder ticks per degree of gate angle
    double angle_open; // angle when gate open
    double angle_closed; // angle when gate closed
    double a_max; // max acceleration
    double v_max; // max velocity
    double v_min; // slow velocity
    int8_t driver_open_dir; // driver pwm sign for open direction. 1 or -1.
    uint16_t max_pwm; // max driver pwm
    double pid_kp;
    double pid_ki;
    double pid_slow_kp;
    double pid_slow_ki;
    double pid_imax;
    uint16_t vel_update_tick_num;
    uint16_t zero_vel_timeout;
    double move_uncert_before; // degrees before target when velocity is reduced
    double move_uncert_after; // degrees after target when velocity still set
    double max_angle_follow_error; // max error when gate stopped is detected
    double hold_open_offset; // open preload offset
};

class Gate {
public:
    

    Gate();
    explicit Gate(gate_params params);
    void begin();
    void open();
    void close();
    void toggle();
    void stop();
    GateState get_state();
    const char *get_state_str() { return state_2_str(state); }
    float get_angle();
    void loop();
    void set_driver(Driver *driver);
    void set_latch(Latch *latch);
    ErrorCode get_error_code();
    const char *get_error_str() { return error_2_str(error_code); }
    void reset();

private:
    struct move {
        uint8_t profile_type;
        double target;
        double t0;
        double s0;
        double t1;
        double s1;
        double t2;
        double s2;
        double t3;
        double s3;
        double t4;
        double s4;
        double v_max;
        double v_min;
        double a_max;

    public:
        enum class move_status {
            inactive = 0,
            in_progress = 1,
            stopped_expected = 2,
            stopped_before_expected = 3,
            not_stopped_expected = 4
        } status;

    };

    // private variables
    uint32_t time = 0; // internal gate time in ms
    double time_sec = 0.; // internal gate time in seconds
    GateState state = GateState::closed;
    GateState prev_state = GateState::closed;
    Driver *driver = nullptr;
    Pid *pid;
    Latch *latch = nullptr;
    bool initialized = false;
    bool motor_enabled = false;
    double angle = 0.0;
    double angle_offset = 0.0;
    double velocity = 0.0;
    double angle_setpoint = 0.0;
    move active_move = {};
    int32_t last_vel_tick = 0;
    uint32_t last_tick_change_time = 0;
    uint8_t move_state_ctrl = 0;
    int8_t open_dir = 1;

    /*
     * Error codes:
     * 0 - no error
     * 1 - gate stopped before expected position
     * 2 - gate did not stop at expected position
     * 3 - stopped manually
     */
    ErrorCode error_code = ErrorCode::no_error;

    // private functions
    void move_(double target);
    void set_pwm_(int16_t pwm);
    void set_pid_(double kp, double ki);
    void enable_motor_();
    void disable_motor_();


    // parameters
    uint16_t loop_dt = 10; // milliseconds between loops

    double enc_ticks_per_deg = 2.725; // encoder ticks per degree of gate angle
    double angle_open = -80.0; // angle when gate open
    double angle_closed = 0.0; // angle when gate closed

    double target_velocity = 15.0; // target opening/closing speed in deg/s
    double target_velocity_slow = 7.5; // final movement reduced velocity

    int8_t driver_open_dir = -1; // driver pwm sign for open direction. 1 or -1.
    uint16_t max_pwm = 150; // max driver pwm

    double pid_kp = 15;
    double pid_ki = 4;
    double pid_slow_kp = 10;
    double pid_slow_ki = 2;
    double pid_imax = 300;

    uint16_t vel_update_tick_num = 5;
    uint16_t zero_vel_timeout = 2000;

    double move_uncert_before = 20.0; // degrees before target when velocity is reduced
    double move_uncert_after = 20.0; // degrees after target when velocity still set

    double max_angle_follow_error = 10.0; // max error when gate stopped is detected
    double hold_open_offset = 5.0;

    double v_max_ = 12;
    double v_min_ = 4;
    double a_max_ = 3;
};


#endif //HOVERGATE_V2_FW_GATE_H