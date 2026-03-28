#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:
    double p_error;
    double i_error;
    double d_error;
    double prev_cte;
    bool is_initialized;

    double Kp;
    double Ki;
    double Kd;

    double output_lim_max;
    double output_lim_min;
    double delta_time;

    PID();
    virtual ~PID();

    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);
    void UpdateError(double cte);
    double TotalError();
    double UpdateDeltaTime(double new_delta_time);
};

#endif // PID_CONTROLLER_H