#include "pid.h"

/**
 * Default constructor. Sets gains all gains to 0.0.
 */
Pid::Pid() {
    set_parameters(0, 0, 0, 1);
};

/**
 * Constructor
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param dt Time step between pid computations. Gains are scaled by this factor in advance.
 */
Pid::Pid(double kp, double ki, double kd, double dt) {
  Pid::set_parameters(kp, ki, kd, dt);
  this->imax_enabled = false;
  started = false;
}

/**
 * Constructor with imax enabled
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param imax Maximum integral sum
 * @param dt Time step between pid computations [seconds]
 */
Pid::Pid(double kp, double ki, double kd, double imax, double dt) {
  set_parameters(kp, ki, kd, imax, dt);
  started = false;

}

/**
 * Set pid parameters (gains).
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param dt Time step
 */
void Pid::set_parameters(double kp, double ki, double kd, double dt) {
  this->kp = kp;
  this->ki = ki * dt;
  this->kd = kd / dt;
}

/**
 * Set pid parameters (gains).
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 *  * @param imax Maximum integral sum

 * @param dt Time step
 */
void Pid::set_parameters(double kp, double ki, double kd, double imax, double dt) {
    this->kp = kp;
    this->ki = ki * dt;
    this->kd = kd / dt;
    this->imax = imax;
    this->imax_enabled = true;
}

/**
 * Initialize pid. This must be called before calling compute or pid will not work.
 */
void Pid::start() {
  integral = 0.0;
  prev_error = 0.0;
  started = true;
}

/**
 * Stop pid operation.
 */
void Pid::stop() {
    started = false;
}

/**
 * Reset pid state.
 */
void Pid::reset() {
    integral = 0.0;
    prev_error = 0.0;
}

/**
 * Compute pid controller.
 * @param in Input value - current value
 * @param setpoint Reference value - controller target
 * @return control value double
 */
double Pid::compute(double in, double setpoint) {
  if (!started) {
    return 0.0;
  }
  
  double error = setpoint - in;
  
  if (imax_enabled) {
    if (integral < imax && integral > -imax) {
      integral += error;
    }
    else if (error * integral < 0) {
      integral += error;
    }
  }
  
  double dError = error - prev_error;

  return kp * error + ki * integral + kd * dError;
}