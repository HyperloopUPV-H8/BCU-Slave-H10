#pragma once

#include "ST-LIB.hpp"

namespace BCU::Control {
class VelocityControl {
    static constexpr double VELOCITY_SATURATOR_UPPER_LIMIT{100.0};
    static constexpr double VELOCITY_SATURATOR_LOWER_LIMIT{0.0};

    PI<IntegratorType::Trapezoidal> velocity;

   public:
    VelocityControl(double velocity_kp, double velocity_ki, double period);

    void execute(double velocity_error);
    void reset();

    const double &get_desired_q_current() const;
};
}  // namespace BCU::Control