#pragma once

#include "ST-LIB_HIGH/Control/Blocks/PI.hpp"

namespace BCU::Control {

class VelocityControl {
    constexpr static float Q_OUT_UPPER_LIMIT{100.0f};
    constexpr static float Q_OUT_LOWER_LIMIT{0.0f};

    float vel_reference;

    PI<IntegratorType::Trapezoidal> vel_pi;

   public:
    VelocityControl(float kp, float ki, float period)
        : vel_pi(kp, ki, period) {}

    float execute(const float &velocity) {
        vel_pi.input_value = vel_reference - velocity;

        vel_pi.execute();

        float q_out{vel_pi.output_value};
        if (q_out >= Q_OUT_UPPER_LIMIT)
            q_out = Q_OUT_UPPER_LIMIT;
        else if (q_out <= Q_OUT_LOWER_LIMIT)
            q_out = Q_OUT_LOWER_LIMIT;

        return q_out;
    }

    void set_velocity_reference(float new_vel) { vel_reference = new_vel; }

    void reset() { vel_pi.reset(); }

    void set_kp(float new_kp) {
        vel_pi.set_kp(new_kp);
        vel_pi.reset();
    }

    void set_ki(float new_ki) {
        vel_pi.set_ki(new_ki);
        vel_pi.reset();
    }

    void set_period(float new_period) {
        vel_pi.set_period(new_period);
        vel_pi.reset();
    }
};

}  // namespace BCU::Control