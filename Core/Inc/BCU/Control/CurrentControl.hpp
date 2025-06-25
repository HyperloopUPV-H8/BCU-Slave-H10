#pragma once

#include "BCU/Control/VoltageGenerator.hpp"
#include "BCU/Sensors/CurrentSense.hpp"
#include "ST-LIB.hpp"

namespace BCU::Control {

struct StaticThreePhase {
    double u{0.0f};
    double v{0.0f};
    double w{0.0f};

    StaticThreePhase(double u, double v, double w) : u(u), v(v), w(w) {}
};

struct StaticTwoPhase {
    double alpha{0.0f};
    double beta{0.0f};
    double zero{0.0f};

    StaticTwoPhase(double alpha, double beta, double zero = 0.0f)
        : alpha(alpha), beta(beta), zero(zero) {}
};

struct RotatingTwoPhase {
    double d{0.0f};
    double q{0.0f};
    double zero{0.0f};

    RotatingTwoPhase(double d, double q, double zero = 0.0f)
        : d(d), q(q), zero(zero) {}
};

StaticTwoPhase clarke_transform(const StaticThreePhase &three_phase);
StaticThreePhase inverse_clarke_transform(const StaticTwoPhase &two_phase);

RotatingTwoPhase park_transform(const StaticTwoPhase &two_phase,
                                double angle_rad);
StaticTwoPhase inverse_park_transform(const RotatingTwoPhase &two_phase,
                                      double angle_rad);

class CurrentControl {
   public:
    constexpr static double d_current_control_period_s{0.001f};
    constexpr static double q_current_control_period_s{0.001f};

   private:
    constexpr static double d_current_control_kp{1.0f};
    constexpr static double d_current_control_ki{5.0f};
    constexpr static double d_current_max_output{220.0f};
    constexpr static double d_current_min_output{-d_current_max_output};

    constexpr static double q_current_control_kp{1.0f};
    constexpr static double q_current_control_ki{5.0f};
    constexpr static double q_current_max_output{220.0f};
    constexpr static double q_current_min_output{-q_current_max_output};

    VoltageGenerator &voltage_generator;
    Sensors::CurrentSense &current_sense;

    PI<IntegratorType::Trapezoidal> d_current_control{
        d_current_control_kp,
        d_current_control_ki,
        d_current_control_period_s,
    };

    PI<IntegratorType::Trapezoidal> q_current_control{
        q_current_control_kp,
        q_current_control_ki,
        q_current_control_period_s,
    };

    double alpha_current_measured{0.0f};
    double beta_current_measured{0.0f};

    double d_current_measured{0.0f};
    double q_current_measured{0.0f};

    double d_current_error{0.0f};
    double d_current_reference{0.0f};

    double q_current_error{0.0f};
    double q_current_reference{0.0f};

    double d_voltage_target{0.0f};
    double q_voltage_target{0.0f};

    double alpha_voltage_target{0.0f};
    double beta_voltage_target{0.0f};

   public:
    CurrentControl(VoltageGenerator &voltage_generator,
                   Sensors::CurrentSense &current_sense);

    void update(const double &electrical_angle_rad);

    void reset();

    double *get_alpha_current_measured_ptr();
    double *get_beta_current_measured_ptr();

    double *get_d_current_measured_ptr();
    double *get_q_current_measured_ptr();

    double *get_d_current_reference_ptr();
    double *get_d_voltage_target_ptr();

    double *get_q_current_reference_ptr();
    double *get_q_voltage_target_ptr();

    double *get_d_current_error_ptr();
    double *get_q_current_error_ptr();

    double *get_alpha_voltage_target_ptr();
    double *get_beta_voltage_target_ptr();
};
};  // namespace BCU::Control
