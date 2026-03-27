/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <algorithm>

PID::PID() : p_error(0.0),
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

void PID::Init(double Kpi, double Ki, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * Initialize PID coefficients and reset errors
   **/
   Kp = Kpi;
   this->Ki = Ki;
   this->Kd = Kdi;

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
   /**
   * Update PID errors based on cte.
   **/
   p_error = cte;

   if (!is_initialized) {
      d_error = 0.0;
      prev_cte = cte;
      is_initialized = true;
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
   * Calculate and return the PID control output with clamping.
   * Output is constrained to [output_lim_min, output_lim_max].
   */
   double raw_control = -(Kp * p_error + Ki * i_error + Kd * d_error);
   double control = std::clamp(raw_control, output_lim_min, output_lim_max);

   // Anti-windup: if saturated, reduce the effective integral
   if (control != raw_control && delta_time > 0.0) {
      double error_sign = (p_error > 0.0 ? 1.0 : (p_error < 0.0 ? -1.0 : 0.0));
      i_error -= error_sign * Ki * delta_time; // soft correction for next iteration
   }

   return control;
}


double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * Update the delta time with new value
   */
   delta_time = new_delta_time;
   return delta_time;
}