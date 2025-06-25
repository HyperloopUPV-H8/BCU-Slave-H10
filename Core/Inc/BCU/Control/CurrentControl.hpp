#pragma once

#include "ST-LIB.hpp"

namespace BCU::Control {

class CurrentControl {
    // TODO: UPDATE THESE VALUES
    constexpr static double D_KP{1.0};
    constexpr static double D_KI{5.0};
    constexpr static double D_MAX_OUTPUT{220.0};
    constexpr static double D_MIN_OUTPUT{-D_MAX_OUTPUT};

    constexpr static double Q_KP{1.0};
    constexpr static double Q_KI{5.0};
    constexpr static double Q_MAX_OUTPUT{220.0};
    constexpr static double Q_MIN_OUTPUT{-Q_MAX_OUTPUT};

    PI<IntegratorType::Trapezoidal> d_current_control;
    PI<IntegratorType::Trapezoidal> q_current_control;

    double saturated_d_output{0.0};
    double saturated_q_output{0.0};

    void update_d(double current);
    void update_q(double current);

   public:
    CurrentControl(double update_period);

    void update(double d_current, double q_current);
    void reset();

    const double& get_d_output() const;
    const double& get_q_output() const;

    double* get_d_output_ptr();
    double* get_q_output_ptr();
};

};  // namespace BCU::Control