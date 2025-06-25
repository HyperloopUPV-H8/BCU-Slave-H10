#include "BCU/Control/CurrentControl.hpp"

namespace BCU::Control {

CurrentControl::CurrentControl(double update_period)
    : d_current_control(D_KP, D_KI, update_period),
      q_current_control(Q_KP, Q_KI, update_period) {}

void CurrentControl::update(double d_current, double q_current) {
    update_d(d_current);
    update_q(q_current);
}

void CurrentControl::reset() {
    d_current_control.reset();
    q_current_control.reset();

    saturated_d_output = 0.0;
    saturated_q_output = 0.0;
}

void CurrentControl::update_d(double current) {
    d_current_control.input(current);
    d_current_control.execute();
    saturated_d_output = d_current_control.output_value;
    if (saturated_d_output > D_MAX_OUTPUT) {
        saturated_d_output = D_MAX_OUTPUT;
    } else if (saturated_d_output < D_MIN_OUTPUT) {
        saturated_d_output = D_MIN_OUTPUT;
    }
}

void CurrentControl::update_q(double current) {
    q_current_control.input(current);
    q_current_control.execute();
    saturated_q_output = q_current_control.output_value;
    if (saturated_q_output > Q_MAX_OUTPUT) {
        saturated_q_output = Q_MAX_OUTPUT;
    } else if (saturated_q_output < Q_MIN_OUTPUT) {
        saturated_q_output = Q_MIN_OUTPUT;
    }
}

const double& CurrentControl::get_d_output() const {
    return saturated_d_output;
}

const double& CurrentControl::get_q_output() const {
    return saturated_q_output;
}

double* CurrentControl::get_d_output_ptr() { return &saturated_d_output; }

double* CurrentControl::get_q_output_ptr() { return &saturated_q_output; }

};  // namespace BCU::Control