#include "BCU/Control/CurrentControl.hpp"

namespace BCU::Control {

StaticTwoPhase clarke_transform(const StaticThreePhase &three_phase) {
    return StaticTwoPhase{
        (2.0 * three_phase.u - three_phase.v - three_phase.w) / 3.0,
        M_SQRT3 * (three_phase.v - three_phase.w) / 3.0,
        (three_phase.u + three_phase.v + three_phase.w) / 3.0,
    };
}

StaticThreePhase inverse_clarke_transform(const StaticTwoPhase &two_phase) {
    return StaticThreePhase{
        two_phase.alpha + two_phase.zero,
        M_SQRT3 * two_phase.beta / 2.0 - two_phase.alpha / 2.0 + two_phase.zero,
        two_phase.zero - two_phase.alpha / 2.0 - M_SQRT3 * two_phase.beta / 2.0,
    };
}

RotatingTwoPhase park_transform(const StaticTwoPhase &two_phase,
                                float angle_rad) {
    float angle_sin{sin(angle_rad)};
    float angle_cos{cos(angle_rad)};

    return RotatingTwoPhase{
        angle_cos * two_phase.alpha + angle_sin * two_phase.beta,
        angle_cos * two_phase.beta - angle_sin * two_phase.alpha,
        two_phase.zero,
    };
}

StaticTwoPhase inverse_park_transform(const RotatingTwoPhase &two_phase,
                                      float angle_rad) {
    float angle_sin{sin(angle_rad)};
    float angle_cos{cos(angle_rad)};

    return StaticTwoPhase{
        angle_sin * two_phase.d + angle_cos * two_phase.q,
        angle_sin * two_phase.q - angle_cos * two_phase.d,
        two_phase.zero,
    };
}

CurrentControl::CurrentControl(VoltageGenerator &voltage_gen,
                               Sensors::CurrentSense &current_sense)
    : voltage_generator(voltage_gen), current_sense(current_sense) {}

void CurrentControl::update(const float &electrical_angle_rad) {
    auto [alpha_current_measured, beta_current_measured, zero_current] =
        clarke_transform({*current_sense.get_average_phase_u_current(),
                          *current_sense.get_average_phase_v_current(),
                          *current_sense.get_average_phase_w_current()});

    this->alpha_current_measured = alpha_current_measured;
    this->beta_current_measured = beta_current_measured;

    auto [d_current_measured, q_current_measured, zero_current_measured] =
        park_transform(
            {alpha_current_measured, beta_current_measured, zero_current},
            electrical_angle_rad);

    this->d_current_measured = d_current_measured;
    this->q_current_measured = q_current_measured;

    d_current_error = d_current_reference - d_current_measured;
    q_current_error = q_current_reference - q_current_measured;

    d_current_control.input(d_current_error);
    d_current_control.execute();
    d_voltage_target = d_current_control.output_value > d_current_max_output
                           ? d_current_max_output
                       : d_current_control.output_value < d_current_min_output
                           ? d_current_min_output
                           : d_current_control.output_value;

    q_current_control.input(q_current_error);
    q_current_control.execute();
    q_voltage_target = q_current_control.output_value > q_current_max_output
                           ? q_current_max_output
                       : q_current_control.output_value < q_current_min_output
                           ? q_current_min_output
                           : q_current_control.output_value;

    auto [alpha_voltage_target, beta_voltage_target, zero_voltage_target] =
        inverse_park_transform({d_voltage_target, q_voltage_target, 0.0f},
                               electrical_angle_rad);

    this->alpha_voltage_target = alpha_voltage_target;
    this->beta_voltage_target = beta_voltage_target;

    auto [u_voltage_target, v_voltage_target, w_voltage_target] =
        inverse_clarke_transform(
            {alpha_voltage_target, beta_voltage_target, zero_voltage_target});

    voltage_generator.generate_voltage(u_voltage_target, v_voltage_target,
                                       w_voltage_target);
}

void CurrentControl::reset() {
    d_current_control.reset();
    q_current_control.reset();
}

float *CurrentControl::get_alpha_current_measured_ptr() {
    return &alpha_current_measured;
}

float *CurrentControl::get_beta_current_measured_ptr() {
    return &beta_current_measured;
}

float *CurrentControl::get_d_current_measured_ptr() {
    return &d_current_measured;
}

float *CurrentControl::get_q_current_measured_ptr() {
    return &q_current_measured;
}

float *CurrentControl::get_d_current_reference_ptr() {
    return &d_current_reference;
}

float *CurrentControl::get_d_voltage_target_ptr() { return &d_voltage_target; }

float *CurrentControl::get_q_current_reference_ptr() {
    return &q_current_reference;
}

float *CurrentControl::get_q_voltage_target_ptr() { return &q_voltage_target; }

float *CurrentControl::get_d_current_error_ptr() { return &d_current_error; }

float *CurrentControl::get_q_current_error_ptr() { return &q_current_error; }

float *CurrentControl::get_alpha_voltage_target_ptr() {
    return &alpha_voltage_target;
}

float *CurrentControl::get_beta_voltage_target_ptr() {
    return &beta_voltage_target;
}

};  // namespace BCU::Control