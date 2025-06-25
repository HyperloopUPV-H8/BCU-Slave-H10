#include "BCU/Control/Executor.hpp"

namespace BCU::Control {

Executor::Executor(Actuators::MotorDriver &motor_driver)
    : motor_driver(motor_driver) {}

void Executor::configure_current_control(float current_d_ref,
                                         float current_q_ref) {}

void Executor::configure_speed_control(float speed_ref) {}

void Executor::start_selected_control() {}

void Executor::stop() {
    switch (mode) {
        case ControlMode::IDLE:
            break;
        case ControlMode::TEST_PWM:
            break;
        case ControlMode::SPACE_VECTOR:
            Time::unregister_mid_precision_alarm(space_vector_alarm_id);
            break;
    }

    stop_motor_driver();

    duty_cycle_u = 0.0f;
    duty_cycle_v = 0.0f;
    duty_cycle_w = 0.0f;

    update_duty_cycle();

    mode = ControlMode::IDLE;
}

void Executor::start_test_pwm(float duty_cycle_u, float duty_cycle_v,
                              float duty_cycle_w) {
    if (mode != ControlMode::IDLE) {
        return;
    }

    mode = ControlMode::TEST_PWM;

    set_duty_cycle_u(duty_cycle_u);
    set_duty_cycle_v(duty_cycle_v);
    set_duty_cycle_w(duty_cycle_w);

    start_motor_driver();
    update_duty_cycle();
}

void Executor::start_space_vector(float modulation_index,
                                  float modulation_frequency_hz) {
    if (mode != ControlMode::IDLE) {
        return;
    }

    mode = ControlMode::SPACE_VECTOR;

    set_modulation_index(modulation_index);
    set_modulation_frequency_hz(modulation_frequency_hz);

    start_motor_driver();

    space_vector_alarm_id =
        Time::register_mid_precision_alarm(space_vector_period_us, [&]() {
            space_vector_time += space_vector_period_us / 1e6f;

            float voltage_u{this->modulation_index *
                            sinf(2.0 * M_PI * this->modulation_frequency_hz *
                                 space_vector_time)};
            float voltage_v{this->modulation_index *
                            sinf(2.0 * M_PI * this->modulation_frequency_hz *
                                     space_vector_time +
                                 2.0 * M_PI / 3.0)};
            float voltage_w{this->modulation_index *
                            sinf(2.0 * M_PI * this->modulation_frequency_hz *
                                     space_vector_time -
                                 2.0 * M_PI / 3.0)};

            float offset = -(std::max({voltage_u, voltage_v, voltage_w}) +
                             std::min({voltage_u, voltage_v, voltage_w})) /
                           2.0f;

            this->duty_cycle_u = 100.0f * (voltage_u + offset + 1.0f) / 2.0f;
            this->duty_cycle_v = 100.0f * (voltage_v + offset + 1.0f) / 2.0f;
            this->duty_cycle_w = 100.0f * (voltage_w + offset + 1.0f) / 2.0f;

            update_duty_cycle();
        });
}

void Executor::set_modulation_index(float modulation_index) {
    if (mode != ControlMode::SPACE_VECTOR) {
        return;
    }
    this->modulation_index = modulation_index;
}

void Executor::set_modulation_frequency_hz(float modulation_frequency_hz) {
    if (mode != ControlMode::SPACE_VECTOR) {
        return;
    }
    this->modulation_frequency_hz = modulation_frequency_hz;
}

void Executor::start_motor_driver() { motor_driver.turn_on(); }

void Executor::stop_motor_driver() { motor_driver.turn_off(); }

void Executor::update_duty_cycle() {
    motor_driver.set_duty_cycle_u(duty_cycle_u);
    motor_driver.set_duty_cycle_v(duty_cycle_v);
    motor_driver.set_duty_cycle_w(duty_cycle_w);
}

void Executor::set_duty_cycle_u(float duty_cycle) {
    if (mode != ControlMode::TEST_PWM) {
        return;
    }
    duty_cycle_u = duty_cycle;
    update_duty_cycle();
}

void Executor::set_duty_cycle_v(float duty_cycle) {
    if (mode != ControlMode::TEST_PWM) {
        return;
    }
    duty_cycle_v = duty_cycle;
    update_duty_cycle();
}

void Executor::set_duty_cycle_w(float duty_cycle) {
    if (mode != ControlMode::TEST_PWM) {
        return;
    }
    duty_cycle_w = duty_cycle;
    update_duty_cycle();
}

float *Executor::get_duty_cycle_u_ptr() { return &duty_cycle_u; }
float *Executor::get_duty_cycle_v_ptr() { return &duty_cycle_v; }
float *Executor::get_duty_cycle_w_ptr() { return &duty_cycle_w; }

};  // namespace BCU::Control