#include "BCU/Control/CurrentControl.hpp"

namespace BCU::Control {

CurrentControl::CurrentControl(double d_current_kp, double d_current_ki,
                               double q_current_kp, double q_current_ki,
                               double period)
    : d_current(d_current_kp, d_current_ki, period),
      q_current(q_current_kp, q_current_ki, period) {}

void CurrentControl::execute(double d_current_error, double q_current_error) {
    d_current.input(d_current_error);
    q_current.input(q_current_error);

    d_current.execute();
    q_current.execute();

    if (d_current.output_value > D_SATURATOR_UPPER_LIMIT) {
        d_current.output_value = D_SATURATOR_UPPER_LIMIT;
    } else if (d_current.output_value < D_SATURATOR_LOWER_LIMIT) {
        d_current.output_value = D_SATURATOR_LOWER_LIMIT;
    }

    if (q_current.output_value > Q_SATURATOR_UPPER_LIMIT) {
        q_current.output_value = Q_SATURATOR_UPPER_LIMIT;
    } else if (q_current.output_value < Q_SATURATOR_LOWER_LIMIT) {
        q_current.output_value = Q_SATURATOR_LOWER_LIMIT;
    }
}

void CurrentControl::reset() {
    d_current.reset();
    q_current.reset();
}

const double &CurrentControl::get_desired_d_voltage() const {
    return d_current.output_value;
}

const double &CurrentControl::get_desired_q_voltage() const {
    return q_current.output_value;
}

};  // namespace BCU::Control
