#include "BCU/Control/Executor.hpp"

namespace BCU::Control {

Executor::Executor(Actuators::MotorDriver &motor_driver)
    : motor_driver(motor_driver) {}

void Executor::stop() {
    stop_motor_driver();

    duty_cycle_u = 0.0f;
    duty_cycle_v = 0.0f;
    duty_cycle_w = 0.0f;

    update_duty_cycle();

    switch (mode) {
        case ControlMode::IDLE:
            break;
        case ControlMode::TEST_PWM:
            break;
    }

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
}

void Executor::set_duty_cycle_v(float duty_cycle) {
    if (mode != ControlMode::TEST_PWM) {
        return;
    }
    duty_cycle_v = duty_cycle;
}

void Executor::set_duty_cycle_w(float duty_cycle) {
    if (mode != ControlMode::TEST_PWM) {
        return;
    }
    duty_cycle_w = duty_cycle;
}

float *Executor::get_duty_cycle_u_ptr() { return &duty_cycle_u; }
float *Executor::get_duty_cycle_v_ptr() { return &duty_cycle_v; }
float *Executor::get_duty_cycle_w_ptr() { return &duty_cycle_w; }

};  // namespace BCU::Control