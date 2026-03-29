/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 * Created on: December 11, 2020
 * Author: Mathilde Badoual
 * Updated: 2026 - Fixed sign, delta_time handling, and C++17 compatibility
 **********************************************/

#include "pid_controller.h"
#include <algorithm>

/**
 * PID Controller Class
 *
 * Implements a classic Proportional-Integral-Derivative (PID) controller
 * commonly used in autonomous vehicle control systems for steering, throttle,
 * and speed control.
 */

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

/**
 * Default destructor
 */
PID::~PID() {}

/**
 * Initialize the PID controller with gains and output limits.
 *
 * @param Kpi Proportional gain (Kp)
 * @param Kii Integral gain (Ki)
 * @param Kdi Derivative gain (Kd)
 * @param output_lim_maxi Maximum allowed control output
 * @param output_lim_mini Minimum allowed control output
 */
void PID::Init(double Kpi, double Kii, double Kdi,
               double output_lim_maxi, double output_lim_mini) {

    Kp = Kpi;
    Ki = Kii;
    Kd = Kdi;

    // Ensure max limit is actually larger than min limit
    output_lim_max = std::max(output_lim_maxi, output_lim_mini);
    output_lim_min = std::min(output_lim_maxi, output_lim_mini);

    // Reset all errors and state
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    prev_cte = 0.0;
    delta_time = 0.0;
    is_initialized = true;
}

/**
 * Update the PID errors based on the current Cross Track Error (CTE).
 *
 * CTE represents the lateral distance from the desired path (positive = right of center).
 * This function calculates:
 *   - Proportional error (current CTE)
 *   - Integral error (accumulated error over time)
 *   - Derivative error (rate of change of error)
 *
 * @param cte Current cross-track error
 */
void PID::UpdateError(double cte) {
    p_error = cte;  // Proportional term

    // First-time initialization: no previous error to compute derivative
    if (!is_initialized) {
        prev_cte = cte;
        is_initialized = true;
        d_error = 0.0;
        return;
    }

    // Only compute derivative and integral if we have a valid time step
    if (delta_time > 0.0) {
        d_error = (cte - prev_cte) / delta_time;        // Derivative term
        i_error += cte * delta_time;                    // Integral term
    } else {
        d_error = 0.0;  // No valid dt → no derivative
    }

    prev_cte = cte;  // Store current error for next derivative calculation
}

/**
 * Calculate the total PID control output with output limiting and anti-windup.
 *
 * The control output is computed as:
 *     control = Kp * p_error + Ki * i_error + Kd * d_error
 *
 * Positive CTE produces positive steering correction (as required by this project).
 *
 * @return Clamped control output (steering angle, throttle, etc.)
 */
double PID::TotalError() {
    /**
     * Calculate PID control output.
     * Positive CTE should produce positive steering correction for this project.
     */
    double raw_control = Kp * p_error + Ki * i_error + Kd * d_error;

    // Clamp the output to the allowed range (manual implementation for C++11/14 compatibility)
    double control = raw_control;
    if (control > output_lim_max) {
        control = output_lim_max;
    } else if (control < output_lim_min) {
        control = output_lim_min;
    }

    // Simple anti-windup: prevent integral windup when output is saturated
    // Back-calculate the integral term to avoid accumulating error while clamped
    if (control != raw_control && delta_time > 0.0 && Ki != 0.0) {
        double excess = raw_control - control;
        i_error -= excess / Ki * delta_time;   // Adjust integral to match clamped output
    }

    return control;
}

/**
 * Update the time delta (dt) used for integral and derivative calculations.
 *
 * This is important for real-time systems where the control loop frequency
 * may vary. Only positive values are accepted.
 *
 * @param new_delta_time Time elapsed since the last update (in seconds)
 * @return The currently used delta_time
 */
double PID::UpdateDeltaTime(double new_delta_time) {
    if (new_delta_time > 0.0) {
        delta_time = new_delta_time;
    }
    return delta_time;
}