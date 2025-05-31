#pragma once

#include "BCU/Control/VoltageGenerator.hpp"
#include "BCU/Sensors/CurrentSense.hpp"
#include "ST-LIB.hpp"

namespace BCU::Control {

struct StaticThreePhase {
    float u{0.0f};
    float v{0.0f};
    float w{0.0f};

    StaticThreePhase(float u, float v, float w) : u(u), v(v), w(w) {}
};

struct StaticTwoPhase {
    float alpha{0.0f};
    float beta{0.0f};
    float zero{0.0f};

    StaticTwoPhase(float alpha, float beta, float zero = 0.0f)
        : alpha(alpha), beta(beta), zero(zero) {}
};

struct RotatingTwoPhase {
    float d{0.0f};
    float q{0.0f};
    float zero{0.0f};

    RotatingTwoPhase(float d, float q, float zero = 0.0f)
        : d(d), q(q), zero(zero) {}
};

StaticTwoPhase clarke_transform(const StaticThreePhase &three_phase);
StaticThreePhase inverse_clarke_transform(const StaticTwoPhase &two_phase);

RotatingTwoPhase park_transform(const StaticTwoPhase &two_phase,
                                float angle_rad);
StaticTwoPhase inverse_park_transform(const RotatingTwoPhase &two_phase,
                                      float angle_rad);

class CurrentControl {
   public:
    constexpr static double d_current_control_period_s{0.001f};
    constexpr static double q_current_control_period_s{0.001f};

   private:
    constexpr static float d_current_control_kp{1.0f};
    constexpr static float d_current_control_ki{5.0f};
    constexpr static float d_current_max_output{220.0f};
    constexpr static float d_current_min_output{-d_current_max_output};

    constexpr static float q_current_control_kp{1.0f};
    constexpr static float q_current_control_ki{5.0f};
    constexpr static float q_current_max_output{220.0f};
    constexpr static float q_current_min_output{-q_current_max_output};

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

    float alpha_current_measured{0.0f};
    float beta_current_measured{0.0f};

    float d_current_measured{0.0f};
    float q_current_measured{0.0f};

    float d_current_error{0.0f};
    float d_current_reference{0.0f};

    float q_current_error{0.0f};
    float q_current_reference{0.0f};

    float d_voltage_target{0.0f};
    float q_voltage_target{0.0f};

    float alpha_voltage_target{0.0f};
    float beta_voltage_target{0.0f};

   public:
    CurrentControl(VoltageGenerator &voltage_generator,
                   Sensors::CurrentSense &current_sense);

    void update(const float &electrical_angle_rad);

    void reset();

    float *get_alpha_current_measured_ptr();
    float *get_beta_current_measured_ptr();

    float *get_d_current_measured_ptr();
    float *get_q_current_measured_ptr();

    float *get_d_current_reference_ptr();
    float *get_d_voltage_target_ptr();

    float *get_q_current_reference_ptr();
    float *get_q_voltage_target_ptr();

    float *get_d_current_error_ptr();
    float *get_q_current_error_ptr();

    float *get_alpha_voltage_target_ptr();
    float *get_beta_voltage_target_ptr();
};
};  // namespace BCU::Control
