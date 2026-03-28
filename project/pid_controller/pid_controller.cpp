/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 *  Modified: 2026 - Fixed sign, delta_time handling, and robustness
 **********************************************/

#include "pid_controller.h"
#include <algorithm>

PID::PID()
    : p_error(0.0),
      i_error(0.0),
      d_error(0.0),
      prev_cte(0.0),
      is_initialized(false),
      Kp(0.0),
      Ki(0.0),
      Kd(0.0),
      output_lim_max(0.0),
      output_lim_min(0.0),
      delta_time(0.0) {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi,
               double output_lim_maxi, double output_lim_mini) {
    /**
     * Initialize PID coefficients and reset all errors
     **/
    Kp = Kpi;
    Ki = Kii;
    Kd = Kdi;

    // Ensure limits are ordered correctly
    output_lim_max = std::max(output_lim_maxi, output_lim_mini);
    output_lim_min = std::min(output_lim_maxi, output_lim_mini);

    // Reset errors
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    prev_cte = 0.0;
    delta_time = 0.0;
    is_initialized = true;
}

void PID::UpdateError(double cte) {
    /**
     * Update PID errors based on cross-track error (cte).
     **/
    p_error = cte;

    if (!is_initialized) {
        // First call ever
        prev_cte = cte;
        is_initialized = true;
        d_error = 0.0;
        return;
    }

    // Update derivative and integral safely
    if (delta_time > 0.0) {
        d_error = (cte - prev_cte) / delta_time;
        i_error += cte * delta_time;
    } else {
        // First frame or invalid delta_time → no derivative, no integral update
        d_error = 0.0;
    }

    prev_cte = cte;
}

double PID::TotalError() {
    /**
     * Calculate the total PID control output with clamping.
     Positive CTE should produce positive steering correction.
     **/
    double raw_control = Kp * p_error + Ki * i_error + Kd * d_error;

    // Clamp the output to allowed steering range
    double control = std::clamp(raw_control, output_lim_min, output_lim_max);

    // Simple anti-windup: if we are saturated, prevent integral from growing further
    if (control != raw_control && delta_time > 0.0) {
        // Back-calculate to reduce integral windup
        double excess = raw_control - control;
        i_error -= excess / Ki * delta_time;   // approximate correction
        // Optional: you can also clamp i_error directly if you prefer
    }

    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
    /**
     * Update the delta time with new value (called every simulation step)
     **/
    if (new_delta_time > 0.0) {
        delta_time = new_delta_time;
    }
    return delta_time;
}