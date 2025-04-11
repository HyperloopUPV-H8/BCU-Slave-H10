#include "BCU/Control/Executor.hpp"

namespace BCU::Control {

void Executor::modulate_output_voltages() {
    auto [target_u, target_v, target_w] =
        RotatingTwoPhaseSystem{d_target_voltage, q_target_voltage}
            .inverse_clarke_park_transform(electrical_angle);

    u_target_voltage = target_u;
    v_target_voltage = target_v;
    w_target_voltage = target_w;

    float offset{0.0};
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

    float dc_link_voltage{motor_driver_sensors.get_dc_link_voltage()};

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

void Executor::start_test_pwm(float duty_cycle_u, float duty_cycle_v,
                              float duty_cycle_w) {
    if (control_mode != ControlMode::IDLE) return;

    control_mode = ControlMode::TEST_PWM;
    modulation_mode = ModulationMode::NONE;

    start_motor_driver();
    set_duty_cycle_u(duty_cycle_u);
    set_duty_cycle_v(duty_cycle_v);
    set_duty_cycle_w(duty_cycle_w);
}

void Executor::stop_test_pwm() {
    if (control_mode != ControlMode::TEST_PWM) return;

    control_mode = ControlMode::IDLE;
    modulation_mode = ModulationMode::NONE;
}

void Executor::set_duty_cycle_u(float duty_cycle) {
    if (control_mode != ControlMode::TEST_PWM) return;

    u_duty_cycle = duty_cycle;
    motor_driver.set_u_duty_cycle(u_duty_cycle);
}

void Executor::set_duty_cycle_v(float duty_cycle) {
    if (control_mode != ControlMode::TEST_PWM) return;

    v_duty_cycle = duty_cycle;
    motor_driver.set_v_duty_cycle(v_duty_cycle);
}

void Executor::set_duty_cycle_w(float duty_cycle) {
    if (control_mode != ControlMode::TEST_PWM) return;

    w_duty_cycle = duty_cycle;
    motor_driver.set_w_duty_cycle(w_duty_cycle);
}

void Executor::start_emulated_movement(float d_current_reference,
                                       float q_current_reference,
                                       float angular_velocity) {
    if (control_mode != ControlMode::IDLE) return;

    control_mode = ControlMode::EMULATED_MOVEMENT;
    modulation_mode = default_modulation_mode;

    set_d_current_reference(d_current_reference);
    set_q_current_reference(q_current_reference);
    set_angluar_velocity(angular_velocity);

    start_motor_driver();
    emulated_movement_alarm_id =
        Time::register_mid_precision_alarm(emulated_movement_period_us, [&]() {
            electrical_angle +=
                angular_velocity * emulated_movement_period_us / 1e6;

            modulate_output_voltages();
        });
}

void Executor::stop_emulated_movement() {
    if (control_mode != ControlMode::EMULATED_MOVEMENT) return;

    control_mode = ControlMode::IDLE;
    modulation_mode = ModulationMode::NONE;

    Time::unregister_mid_precision_alarm(emulated_movement_alarm_id);
    emulated_movement_alarm_id = UNDEFINED_ALARM_ID;
}

void Executor::set_d_current_reference(float current_reference) {
    if (control_mode != ControlMode::EMULATED_MOVEMENT &&
        control_mode != ControlMode::CURRENT_CONTROL)
        return;

    d_current_reference = current_reference;
}

void Executor::set_q_current_reference(float current_reference) {
    if (control_mode != ControlMode::EMULATED_MOVEMENT &&
        control_mode != ControlMode::CURRENT_CONTROL)
        return;

    q_current_reference = current_reference;
}

void Executor::set_angluar_velocity(float angular_velocity) {
    if (control_mode != ControlMode::EMULATED_MOVEMENT) return;

    this->angular_velocity = angular_velocity;
}

void Executor::read_electrical_angle() {
    float position = *position_encoder.get_position();
    electrical_angle = position / 0.096;
    electrical_angle -= 2.0 * std::floor(electrical_angle / 2.0);
    electrical_angle *= M_PI;
}

void Executor::current_control_loop() {
    read_electrical_angle();

    u_current_measurement = motor_driver_sensors.get_motor_phase_u_current();
    v_current_measurement = motor_driver_sensors.get_motor_phase_v_current();
    w_current_measurement = motor_driver_sensors.get_motor_phase_w_current();

    auto [d_current, q_current, zero] =
        ThreePhaseSystem{u_current_measurement, v_current_measurement,
                         w_current_measurement}
            .clarke_park_transform(electrical_angle);

    d_current_measurement = d_current;
    q_current_measurement = q_current;
    three_phase_unbalance = zero;

    d_current_error = d_current_reference - d_current_measurement;
    q_current_error = q_current_reference - q_current_measurement;

    current_control.execute(d_current_error, q_current_error);

    d_target_voltage = current_control.get_desired_d_voltage();
    q_target_voltage = current_control.get_desired_q_voltage();

    modulate_output_voltages();
}

void Executor::start_current_control(float d_current_reference,
                                     float q_current_reference) {
    if (control_mode != ControlMode::IDLE) return;

    control_mode = ControlMode::CURRENT_CONTROL;
    modulation_mode = default_modulation_mode;

    set_d_current_reference(d_current_reference);
    set_q_current_reference(q_current_reference);

    start_motor_driver();
    current_control_alarm_id = Time::register_mid_precision_alarm(
        current_control_period_us, [&]() { current_control_loop(); });
}

void Executor::stop_current_control() {
    if (control_mode != ControlMode::CURRENT_CONTROL) return;

    control_mode = ControlMode::IDLE;
    modulation_mode = ModulationMode::NONE;

    Time::unregister_mid_precision_alarm(current_control_alarm_id);
    current_control_alarm_id = UNDEFINED_ALARM_ID;
}

void Executor::velocity_control_loop() {
    velocity_measurement = *position_encoder.get_velocity();
    velocity_error = velocity_reference - velocity_measurement;

    velocity_control.execute(velocity_error);

    d_current_reference = 0.0;
    q_current_reference = velocity_control.get_desired_q_current();
}

void Executor::start_velocity_control(float velocity_reference) {
    if (control_mode != ControlMode::IDLE) return;

    control_mode = ControlMode::VELOCITY_CONTROL;
    modulation_mode = default_modulation_mode;

    set_velocity_reference(velocity_reference);

    start_motor_driver();
    velocity_control_alarm_id = Time::register_mid_precision_alarm(
        velocity_control_period_us, [&]() { velocity_control_loop(); });

    current_control_alarm_id = Time::register_mid_precision_alarm(
        current_control_period_us, [&]() { current_control_loop(); });
}

void Executor::stop_velocity_control() {
    if (control_mode != ControlMode::VELOCITY_CONTROL) return;

    control_mode = ControlMode::IDLE;
    modulation_mode = ModulationMode::NONE;

    Time::unregister_mid_precision_alarm(velocity_control_alarm_id);
    velocity_control_alarm_id = UNDEFINED_ALARM_ID;

    Time::unregister_mid_precision_alarm(current_control_alarm_id);
    current_control_alarm_id = UNDEFINED_ALARM_ID;
}

void Executor::set_velocity_reference(float velocity_reference) {
    if (control_mode != ControlMode::VELOCITY_CONTROL) return;

    this->velocity_reference = velocity_reference;
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

float *Executor::get_velocity_reference() { return &velocity_reference; }
float *Executor::get_velocity_error() { return &velocity_error; }
float *Executor::get_u_current_measurement() { return &u_current_measurement; }
float *Executor::get_v_current_measurement() { return &v_current_measurement; }
float *Executor::get_w_current_measurement() { return &w_current_measurement; }
float *Executor::get_electrical_angle() { return &electrical_angle; }
float *Executor::get_d_current_reference() { return &d_current_reference; }
float *Executor::get_d_current_measurement() { return &d_current_measurement; }
float *Executor::get_d_current_error() { return &d_current_error; }
float *Executor::get_q_current_reference() { return &q_current_reference; }
float *Executor::get_q_current_measurement() { return &q_current_measurement; }
float *Executor::get_q_current_error() { return &q_current_error; }
float *Executor::get_three_phase_unbalance() { return &three_phase_unbalance; }
float *Executor::get_d_target_voltage() { return &d_target_voltage; }
float *Executor::get_q_target_voltage() { return &q_target_voltage; }
float *Executor::get_u_target_voltage() { return &u_target_voltage; }
float *Executor::get_v_target_voltage() { return &v_target_voltage; }
float *Executor::get_w_target_voltage() { return &w_target_voltage; }
float *Executor::get_u_output_voltage() { return &u_output_voltage; }
float *Executor::get_v_output_voltage() { return &v_output_voltage; }
float *Executor::get_w_output_voltage() { return &w_output_voltage; }
float *Executor::get_u_duty_cycle() { return &u_duty_cycle; }
float *Executor::get_v_duty_cycle() { return &v_duty_cycle; }
float *Executor::get_w_duty_cycle() { return &w_duty_cycle; }
float *Executor::get_angular_velocity() { return &angular_velocity; }

Executor::Executor(Actuators::MotorDriver &motor_driver,
                   Sensors::MotorDriver &motor_driver_sensors,
                   Sensors::PositionEncoder &position_encoder)
    : motor_driver(motor_driver),
      motor_driver_sensors(motor_driver_sensors),
      position_encoder(position_encoder) {}

};  // namespace BCU::Control