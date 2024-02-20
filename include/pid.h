#ifndef PID_H
#define PID_H


class Pid {
  public:
    Pid();
    Pid(double kp, double ki, double kd, double dt);
    Pid(double kp, double ki, double kd, double imax, double dt);
    void set_parameters(double kp, double ki, double kd, double dt);
    void set_parameters(double kp, double ki, double kd, double imax, double dt);
    void start();
    void stop();
    void reset();
    double compute(double in, double setpoint);
  private:
    bool started;
    double kp;
    double ki;
    double kd;
    double prev_error;
    double integral;
    bool imax_enabled;
    double imax;
};

#endif