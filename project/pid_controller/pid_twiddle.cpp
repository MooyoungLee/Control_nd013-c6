#include "pid_twiddle.h"

#include <algorithm>
#include <iostream>

PidTwiddle::PidTwiddle(const std::string& name,
                       const PidGains& initial_gains,
                       const PidGains& initial_deltas,
                       double tolerance,
                       int warmup_steps,
                       int evaluation_steps,
                       bool enabled)
    : name_(name),
      gains_(initial_gains),
      parameters_{initial_gains.kp, initial_gains.ki, initial_gains.kd},
      deltas_{initial_deltas.kp, initial_deltas.ki, initial_deltas.kd},
      tolerance_(tolerance),
      warmup_steps_(warmup_steps),
      evaluation_steps_(evaluation_steps),
      enabled_(enabled),
      has_best_error_(false),
      best_error_(0.0),
      accumulated_error_(0.0),
      steps_in_window_(0),
      parameter_index_(0),
      phase_(Phase::kIncrease) {}

const PidGains& PidTwiddle::gains() const {
    return gains_;
}

bool PidTwiddle::enabled() const {
    return enabled_;
}

bool PidTwiddle::converged() const {
    return SumDeltas() <= tolerance_;
}

bool PidTwiddle::Update(double error) {
    if (!enabled_ || evaluation_steps_ <= 0) {
        return false;
    }

    ++steps_in_window_;
    if (steps_in_window_ > warmup_steps_) {
        accumulated_error_ += error * error;
    }

    if (steps_in_window_ < warmup_steps_ + evaluation_steps_) {
        return false;
    }

    const double average_error = accumulated_error_ / static_cast<double>(evaluation_steps_);

    if (!has_best_error_) {
        best_error_ = average_error;
        has_best_error_ = true;
        std::cout << "[Twiddle:" << name_ << "] baseline error = " << best_error_ << std::endl;
        ApplyCurrentParameterTrial();
        ResetEvaluationWindow();
        return true;
    }

    if (phase_ == Phase::kIncrease) {
        if (average_error < best_error_) {
            best_error_ = average_error;
            deltas_[parameter_index_] *= 1.1;
            AdvanceToNextParameter();
        } else {
            parameters_[parameter_index_] =
                ClampNonNegative(parameters_[parameter_index_] - 2.0 * deltas_[parameter_index_]);
            phase_ = Phase::kDecrease;
            SyncGainsFromParameters();
        }
    } else {
        if (average_error < best_error_) {
            best_error_ = average_error;
            deltas_[parameter_index_] *= 1.1;
        } else {
            parameters_[parameter_index_] =
                ClampNonNegative(parameters_[parameter_index_] + deltas_[parameter_index_]);
            deltas_[parameter_index_] *= 0.9;
        }
        AdvanceToNextParameter();
    }

    if (converged()) {
        enabled_ = false;
        std::cout << "[Twiddle:" << name_ << "] converged at "
                  << "kp=" << gains_.kp << ", ki=" << gains_.ki << ", kd=" << gains_.kd
                  << ", best error=" << best_error_ << std::endl;
    } else {
        std::cout << "[Twiddle:" << name_ << "] kp=" << gains_.kp
                  << ", ki=" << gains_.ki
                  << ", kd=" << gains_.kd
                  << ", best error=" << best_error_ << std::endl;
    }

    ResetEvaluationWindow();
    return true;
}

void PidTwiddle::AdvanceToNextParameter() {
    parameter_index_ = (parameter_index_ + 1) % 3;
    phase_ = Phase::kIncrease;
    ApplyCurrentParameterTrial();
}

void PidTwiddle::ApplyCurrentParameterTrial() {
    parameters_[parameter_index_] += deltas_[parameter_index_];
    SyncGainsFromParameters();
}

void PidTwiddle::ResetEvaluationWindow() {
    accumulated_error_ = 0.0;
    steps_in_window_ = 0;
}

void PidTwiddle::SyncGainsFromParameters() {
    gains_.kp = parameters_[0];
    gains_.ki = parameters_[1];
    gains_.kd = parameters_[2];
}

double PidTwiddle::SumDeltas() const {
    return deltas_[0] + deltas_[1] + deltas_[2];
}

double PidTwiddle::ClampNonNegative(double value) const {
    return std::max(0.0, value);
}
