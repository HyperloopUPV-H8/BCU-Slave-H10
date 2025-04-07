#pragma once

#include "ST-LIB.hpp"

namespace BCU::Control {

class CurrentControl {
    static constexpr double D_SATURATOR_UPPER_LIMIT{300.0};
    static constexpr double D_SATURATOR_LOWER_LIMIT{-300.0};

    static constexpr double Q_SATURATOR_UPPER_LIMIT{300.0};
    static constexpr double Q_SATURATOR_LOWER_LIMIT{-300.0};

    PI<IntegratorType::Trapezoidal> d_current;
    PI<IntegratorType::Trapezoidal> q_current;

   public:
    CurrentControl(double d_current_kp, double d_current_ki,
                   double q_current_kp, double q_current_ki, double period);

    void execute(double d_current_error, double q_current_error);
    void reset();

    const double &get_desired_d_voltage() const;
    const double &get_desired_q_voltage() const;
};

};  // namespace BCU::Control