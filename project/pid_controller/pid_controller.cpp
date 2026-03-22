/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <math.h>

using namespace std;

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

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   Kp = Kpi;
   Ki = Kii;
   Kd = Kdi;
   output_lim_max = output_lim_maxi;
   output_lim_min = output_lim_mini;
   p_error = 0.0;
   i_error = 0.0;
   d_error = 0.0;
   prev_cte = 0.0;
   delta_time = 0.0;
   is_initialized = false;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   p_error = cte;

   if (!is_initialized) {
      d_error = 0.0;
      prev_cte = cte;
      is_initialized = true;
   } else if (delta_time > 0.0) {
      d_error = (cte - prev_cte) / delta_time;
      prev_cte = cte;
   } else {
      d_error = 0.0;
      prev_cte = cte;
   }

   if (delta_time > 0.0) {
      i_error += cte * delta_time;
   }
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = Kp * p_error + Ki * i_error + Kd * d_error;
    if (control > output_lim_max) {
        control = output_lim_max;
    } else if (control < output_lim_min) {
        control = output_lim_min;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   delta_time = new_delta_time;
   return delta_time;
}

Twiddle::Twiddle()
    : tolerance(0.0),
      best_error(std::numeric_limits<double>::infinity()),
      error_sum(0.0),
      settle_frames(0),
      eval_frames(0),
      frame_count(0),
      iteration(0),
      param_index(0),
      stage(kNeedInit),
      initialized(false),
      enabled(false),
      finished(false) {}

void Twiddle::Init(const std::vector<double>& initial_p,
                   const std::vector<double>& initial_dp,
                   double tolerance_in,
                   int settle_frames_in,
                   int eval_frames_in) {
   p = initial_p;
   dp = initial_dp;
   tolerance = tolerance_in;
   settle_frames = settle_frames_in;
   eval_frames = eval_frames_in;
   best_error = std::numeric_limits<double>::infinity();
   error_sum = 0.0;
   frame_count = 0;
   iteration = 0;
   param_index = 0;
   stage = kNeedInit;
   initialized = false;
   enabled = false;
   finished = false;
   for (double delta : dp) {
      if (delta > 0.0) {
         enabled = true;
      }
   }
}

bool Twiddle::IsEnabled() const {
   return enabled;
}

bool Twiddle::IsFinished() const {
   return finished;
}

std::vector<double> Twiddle::GetParams() const {
   return p;
}

std::vector<double> Twiddle::GetDeltaParams() const {
   return dp;
}

double Twiddle::GetBestError() const {
   return best_error;
}

void Twiddle::ResetRun() {
   frame_count = 0;
   error_sum = 0.0;
}

void Twiddle::ClampParams() {
   if (p.size() >= 3) {
      p[0] = std::min(std::max(p[0], 0.0), 2.0);
      p[1] = std::min(std::max(p[1], 0.0), 0.1);
      p[2] = std::min(std::max(p[2], 0.0), 0.5);
   }
}

void Twiddle::ApplyToPid(PID& pid) const {
   pid.Init(p[0], p[1], p[2], pid.output_lim_max, pid.output_lim_min);
}

void Twiddle::AdvanceToNextParameter(PID& pid) {
   double dp_sum = 0.0;
   for (double delta : dp) {
      dp_sum += delta;
   }

   if (dp_sum <= tolerance) {
      enabled = false;
      finished = true;
      std::cout << "Twiddle optimization done. Final gains: "
                << p[0] << ", " << p[1] << ", " << p[2]
                << " with error " << best_error << std::endl;
      return;
   }

   param_index = (param_index + 1) % static_cast<int>(p.size());
   p[param_index] += dp[param_index];
   ClampParams();
   stage = kTryIncrease;
   ApplyToPid(pid);
   ResetRun();
}

bool Twiddle::Update(double cte, PID& pid) {
   if (!enabled || eval_frames <= 0 || std::isnan(cte) || std::isinf(cte)) {
      return false;
   }

   frame_count++;
   if (frame_count > settle_frames) {
      error_sum += cte * cte;
   }

   if (frame_count < settle_frames + eval_frames) {
      return false;
   }

   double err = error_sum / static_cast<double>(eval_frames);

   if (!initialized) {
      best_error = err;
      initialized = true;
      p[param_index] += dp[param_index];
      ClampParams();
      stage = kTryIncrease;
      ApplyToPid(pid);
      ResetRun();
      std::cout << "Twiddle baseline error: " << best_error
                << " starting optimization from gains "
                << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
      return true;
   }

   iteration++;
   std::cout << "Twiddle iteration " << iteration
             << ", param index " << param_index
             << ", error " << err
             << ", best " << best_error
             << ", p = [" << p[0] << ", " << p[1] << ", " << p[2] << "]"
             << ", dp = [" << dp[0] << ", " << dp[1] << ", " << dp[2] << "]"
             << std::endl;

   if (stage == kTryIncrease) {
      if (err < best_error) {
         best_error = err;
         dp[param_index] *= 1.1;
         std::cout << "Twiddle improved with +dp on index " << param_index
                   << ". Error: " << best_error << std::endl;
         AdvanceToNextParameter(pid);
      } else {
         p[param_index] -= 2.0 * dp[param_index];
         ClampParams();
         stage = kTryDecrease;
         ApplyToPid(pid);
         ResetRun();
      }
      return true;
   }

   if (err < best_error) {
      best_error = err;
      dp[param_index] *= 1.1;
      std::cout << "Twiddle improved with -dp on index " << param_index
                << ". Error: " << best_error << std::endl;
   } else {
      p[param_index] += dp[param_index];
      ClampParams();
      dp[param_index] *= 0.9;
   }

   AdvanceToNextParameter(pid);
   return true;
}
