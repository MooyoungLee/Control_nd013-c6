/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 *  Updated: 2026 - Fixed sign, delta_time handling, and C++17 compatibility
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
    Kp = Kpi;
    Ki = Kii;
    Kd = Kdi;

    output_lim_max = std::max(output_lim_maxi, output_lim_mini);
    output_lim_min = std::min(output_lim_maxi, output_lim_mini);

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    prev_cte = 0.0;
    delta_time = 0.0;
    is_initialized = true;
}

void PID::UpdateError(double cte) {
    p_error = cte;

    if (!is_initialized) {
        prev_cte = cte;
        is_initialized = true;
        d_error = 0.0;
        return;
    }

    if (delta_time > 0.0) {
        d_error = (cte - prev_cte) / delta_time;
        i_error += cte * delta_time;
    } else {
        d_error = 0.0;
    }

    prev_cte = cte;
}

double PID::TotalError() {
    /**
     * Calculate PID control output.
     * Positive CTE should produce positive steering correction for this project.
     */
    double raw_control = Kp * p_error + Ki * i_error + Kd * d_error;

    // Manual clamp (std::clamp not available in C++11/C++14)
    double control = raw_control;
    if (control > output_lim_max) {
        control = output_lim_max;
    } else if (control < output_lim_min) {
        control = output_lim_min;
    }

    // Simple anti-windup
    if (control != raw_control && delta_time > 0.0 && Ki != 0.0) {
        double excess = raw_control - control;
        i_error -= excess / Ki * delta_time;   // back-calculate integral
    }

    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
    if (new_delta_time > 0.0) {
        delta_time = new_delta_time;
    }
    return delta_time;
}