#ifndef PID_TWIDDLE_H
#define PID_TWIDDLE_H

#include <string>

struct PidGains {
    double kp;
    double ki;
    double kd;
};

class PidTwiddle {
public:
    PidTwiddle(const std::string& name,
               const PidGains& initial_gains,
               const PidGains& initial_deltas,
               double tolerance,
               int warmup_steps,
               int evaluation_steps,
               bool enabled);

    const PidGains& gains() const;
    bool Update(double error);
    bool enabled() const;
    bool converged() const;

private:
    enum class Phase {
        kIncrease,
        kDecrease
    };

    void AdvanceToNextParameter();
    void ApplyCurrentParameterTrial();
    void ResetEvaluationWindow();
    void SyncGainsFromParameters();
    double SumDeltas() const;
    double ClampNonNegative(double value) const;

    std::string name_;
    PidGains gains_;
    double parameters_[3];
    double deltas_[3];
    double tolerance_;
    int warmup_steps_;
    int evaluation_steps_;
    bool enabled_;
    bool has_best_error_;
    double best_error_;
    double accumulated_error_;
    int steps_in_window_;
    int parameter_index_;
    Phase phase_;
};

#endif  // PID_TWIDDLE_H
