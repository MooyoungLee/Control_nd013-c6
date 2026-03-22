/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <vector>

class PID {
public:

   /**
   * TODO: Create the PID class
   **/

    /*
    * Errors
    */
    double p_error;
    double i_error;
    double d_error;
    double prev_cte;
    bool is_initialized;

    /*
    * Coefficients
    */
    double Kp;
    double Ki;
    double Kd;

    /*
    * Output limits
    */
    double output_lim_max;
    double output_lim_min;
  
    /*
    * Delta time
    */
    double delta_time;

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
  
    /*
    * Update the delta time.
    */
    double UpdateDeltaTime(double new_delta_time);
};

class Twiddle {
public:
    Twiddle();

    void Init(const std::vector<double>& initial_p,
              const std::vector<double>& initial_dp,
              double tolerance,
              int settle_frames,
              int eval_frames);

    bool Update(double cte, PID& pid);
    bool IsEnabled() const;
    bool IsFinished() const;
    std::vector<double> GetParams() const;
    std::vector<double> GetDeltaParams() const;
    double GetBestError() const;

private:
    enum Stage {
        kNeedInit,
        kTryIncrease,
        kTryDecrease
    };

    void ResetRun();
    void ClampParams();
    void ApplyToPid(PID& pid) const;
    void AdvanceToNextParameter(PID& pid);

    std::vector<double> p;
    std::vector<double> dp;
    double tolerance;
    double best_error;
    double error_sum;
    int settle_frames;
    int eval_frames;
    int frame_count;
    int iteration;
    int param_index;
    Stage stage;
    bool initialized;
    bool enabled;
    bool finished;
};

#endif //PID_CONTROLLER_H


