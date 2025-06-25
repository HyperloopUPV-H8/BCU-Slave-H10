#pragma once

#include "ST-LIB.hpp"

namespace BCU::Control {

class SpeedControl {
    constexpr static double KP{6.0};
    constexpr static double KI{15.0};
    constexpr static double MAX_OUTPUT{45.0};
    constexpr static double MIN_OUTPUT{0};

    PI<IntegratorType::Trapezoidal> speed_control;

    float target_linear_speed{0.0};
    float measured_linear_speed{0.0};
    float linear_speed_error{0.0};
    float saturated_pi_output{0.0};

   public:
    SpeedControl(double update_period) : speed_control(KP, KI, update_period) {}
    void reset();
    void update(float measured_linear_speed);
    float* get_target_linear_speed_ptr() { return &target_linear_speed; }
    float* get_measured_linear_speed_ptr() { return &measured_linear_speed; }
    float* get_saturated_pi_output_ptr() { return &saturated_pi_output; }
};
}  // namespace BCU::Control