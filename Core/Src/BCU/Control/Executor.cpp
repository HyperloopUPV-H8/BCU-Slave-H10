#include "BCU/Control/Executor.hpp"

namespace BCU::Control {

void Executor::modulate_output_voltages() {
    auto [target_u, target_v, target_w] =
        RotatingTwoPhaseSystem{d_target_voltage, q_target_voltage}
            .inverse_clarke_park_transform(electrical_angle);

    u_target_voltage = target_u;
    v_target_voltage = target_v;
    w_target_voltage = target_w;

    double offset{0.0};
    if (modulation_mode == ModulationMode::THIRD_HARMONIC) {
        offset =
            -(std::min({u_target_voltage, v_target_voltage, w_target_voltage}) +
              std::max(
                  {u_target_voltage, v_target_voltage, w_target_voltage})) /
            2.0;
    }

    u_output_voltage = u_target_voltage + offset;
    v_output_voltage = v_target_voltage + offset;
    w_output_voltage = w_target_voltage + offset;

    double dc_link_voltage{motor_driver_sensors.get_dc_link_voltage()};

    u_duty_cycle = 100.0 * ((u_output_voltage / dc_link_voltage) + 1.0) / 2.0;
    v_duty_cycle = 100.0 * ((v_output_voltage / dc_link_voltage) + 1.0) / 2.0;
    w_duty_cycle = 100.0 * ((w_output_voltage / dc_link_voltage) + 1.0) / 2.0;

    motor_driver.set_u_duty_cycle(u_duty_cycle);
    motor_driver.set_v_duty_cycle(v_duty_cycle);
    motor_driver.set_w_duty_cycle(w_duty_cycle);
}

void Executor::start_motor_driver() {
    motor_driver.enable_buffer();
    motor_driver.turn_reset_on();
    motor_driver.turn_on_pwms();
}

void Executor::stop_motor_driver() {
    motor_driver.turn_off_pwms();
    motor_driver.turn_reset_off();
    motor_driver.disable_buffer();
}

void Executor::start_test_pwm(double duty_cycle_u, double duty_cycle_v,
                              double duty_cycle_w) {
    if (control_mode != ControlMode::IDLE) return;

    control_mode = ControlMode::TEST_PWM;
    modulation_mode = ModulationMode::NONE;

    set_duty_cycle_u(duty_cycle_u);
    set_duty_cycle_v(duty_cycle_v);
    set_duty_cycle_w(duty_cycle_w);

    start_motor_driver();
}

void Executor::stop_test_pwm() {
    if (control_mode != ControlMode::TEST_PWM) return;

    control_mode = ControlMode::IDLE;
    modulation_mode = ModulationMode::NONE;
}

void Executor::set_duty_cycle_u(double duty_cycle) {
    if (control_mode != ControlMode::TEST_PWM) return;

    u_duty_cycle = duty_cycle;
    motor_driver.set_u_duty_cycle(u_duty_cycle);
}

void Executor::set_duty_cycle_v(double duty_cycle) {
    if (control_mode != ControlMode::TEST_PWM) return;

    v_duty_cycle = duty_cycle;
    motor_driver.set_v_duty_cycle(v_duty_cycle);
}

void Executor::set_duty_cycle_w(double duty_cycle) {
    if (control_mode != ControlMode::TEST_PWM) return;

    w_duty_cycle = duty_cycle;
    motor_driver.set_w_duty_cycle(w_duty_cycle);
}

void Executor::start_emulated_movement(double d_current_reference,
                                       double q_current_reference,
                                       double angular_velocity) {
    if (control_mode != ControlMode::IDLE) return;

    control_mode = ControlMode::EMULATED_MOVEMENT;
    modulation_mode = default_modulation_mode;

    set_d_current_reference(d_current_reference);
    set_q_current_reference(q_current_reference);
    set_angluar_velocity(angular_velocity);

    emulated_movement_alarm_id =
        Time::register_mid_precision_alarm(emulated_movement_period_us, [&]() {
            electrical_angle +=
                angular_velocity * emulated_movement_period_us / 1e6;

            modulate_output_voltages();
        });

    start_motor_driver();
}

void Executor::stop_emulated_movement() {
    if (control_mode != ControlMode::EMULATED_MOVEMENT) return;

    control_mode = ControlMode::IDLE;
    modulation_mode = ModulationMode::NONE;

    Time::unregister_mid_precision_alarm(emulated_movement_alarm_id);
    emulated_movement_alarm_id = UNDEFINED_ALARM_ID;
}

void Executor::set_d_current_reference(double current_reference) {
    if (control_mode != ControlMode::EMULATED_MOVEMENT &&
        control_mode != ControlMode::CURRENT_CONTROL)
        return;

    d_current_reference = current_reference;
}

void Executor::set_q_current_reference(double current_reference) {
    if (control_mode != ControlMode::EMULATED_MOVEMENT &&
        control_mode != ControlMode::CURRENT_CONTROL)
        return;

    q_current_reference = current_reference;
}

void Executor::set_angluar_velocity(double angular_velocity) {
    if (control_mode != ControlMode::EMULATED_MOVEMENT) return;

    this->angular_velocity = angular_velocity;
}

void Executor::read_electrical_angle() {
    double position = position_encoder.get_position();
    electrical_angle = position / 0.096;
    electrical_angle -= 2.0 * std::floor(electrical_angle / 2.0);
    electrical_angle *= M_PI;
}

void Executor::current_control_loop() {
    read_electrical_angle();

    auto [d_current, q_current, zero] =
        ThreePhaseSystem{motor_driver_sensors.get_motor_phase_u_current(),
                         motor_driver_sensors.get_motor_phase_v_current(),
                         motor_driver_sensors.get_motor_phase_w_current()}
            .clarke_park_transform(electrical_angle);

    three_phase_unbalance = zero;

    d_current_error = d_current_reference - d_current;
    q_current_error = q_current_reference - q_current;

    current_control.execute(d_current_error, q_current_error);

    d_target_voltage = current_control.get_desired_d_voltage();
    q_target_voltage = current_control.get_desired_q_voltage();

    modulate_output_voltages();
}

void Executor::start_current_control(double d_current_reference,
                                     double q_current_reference) {
    if (control_mode != ControlMode::IDLE) return;

    control_mode = ControlMode::CURRENT_CONTROL;
    modulation_mode = default_modulation_mode;

    set_d_current_reference(d_current_reference);
    set_q_current_reference(q_current_reference);

    current_control_alarm_id = Time::register_mid_precision_alarm(
        current_control_period_us, [&]() { current_control_loop(); });

    start_motor_driver();
}

void Executor::stop_current_control() {
    if (control_mode != ControlMode::CURRENT_CONTROL) return;

    control_mode = ControlMode::IDLE;
    modulation_mode = ModulationMode::NONE;

    Time::unregister_mid_precision_alarm(current_control_alarm_id);
    current_control_alarm_id = UNDEFINED_ALARM_ID;
}

void Executor::velocity_control_loop() {
    velocity_error = velocity_reference - position_encoder.get_velocity();

    velocity_control.execute(velocity_error);

    d_current_reference = 0.0;
    q_current_reference = velocity_control.get_desired_q_current();
}

void Executor::start_velocity_control(double velocity_reference) {
    if (control_mode != ControlMode::IDLE) return;

    control_mode = ControlMode::VELOCITY_CONTROL;
    modulation_mode = default_modulation_mode;

    set_velocity_reference(velocity_reference);

    velocity_control_alarm_id = Time::register_mid_precision_alarm(
        velocity_control_period_us, [&]() { velocity_control_loop(); });

    current_control_alarm_id = Time::register_mid_precision_alarm(
        current_control_period_us, [&]() { current_control_loop(); });

    start_motor_driver();
}

void Executor::use_sine_modulation() {
    if (control_mode != ControlMode::IDLE) return;

    modulation_mode = ModulationMode::SINE;
}

void Executor::use_third_harmonic_modulation() {
    if (control_mode != ControlMode::IDLE) return;

    modulation_mode = ModulationMode::THIRD_HARMONIC;
}

void Executor::stop() {
    stop_motor_driver();

    stop_test_pwm();
    stop_emulated_movement();
    stop_current_control();
    stop_velocity_control();

    u_duty_cycle = 0.0;
    v_duty_cycle = 0.0;
    w_duty_cycle = 0.0;

    u_output_voltage = 0.0;
    v_output_voltage = 0.0;
    w_output_voltage = 0.0;

    u_target_voltage = 0.0;
    v_target_voltage = 0.0;
    w_target_voltage = 0.0;

    d_target_voltage = 0.0;
    q_target_voltage = 0.0;

    d_current_reference = 0.0;
    d_current_error = 0.0;

    q_current_reference = 0.0;
    q_current_error = 0.0;

    velocity_reference = 0.0;
    velocity_error = 0.0;

    angular_velocity = 0.0;
    electrical_angle = 0.0;

    motor_driver.set_u_duty_cycle(0.0);
    motor_driver.set_v_duty_cycle(0.0);
    motor_driver.set_w_duty_cycle(0.0);
}
};  // namespace BCU::Control