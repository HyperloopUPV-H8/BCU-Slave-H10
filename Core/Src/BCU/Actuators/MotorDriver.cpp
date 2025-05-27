#include "BCU/Actuators/MotorDriver.hpp"

namespace BCU::Actuators {

ThreePhasePWM::ThreePhasePWM(Pin& phase_u_pin, Pin& phase_u_negated_pin,
                             Pin& phase_v_pin, Pin& phase_v_negated_pin,
                             Pin& phase_w_pin, Pin& phase_w_negated_pin)
    : phase_u{DualPWM{phase_u_pin, phase_u_negated_pin}},
      phase_v{DualPWM{phase_v_pin, phase_v_negated_pin}},
      phase_w{DualPWM{phase_w_pin, phase_w_negated_pin}} {}

void ThreePhasePWM::turn_on() {
    phase_u.turn_on();
    phase_v.turn_on();
    phase_w.turn_on();
}

void ThreePhasePWM::turn_off() {
    phase_u.turn_off();
    phase_v.turn_off();
    phase_w.turn_off();
}

void ThreePhasePWM::set_duty_cycle_u(float duty_cycle) {
    phase_u.set_duty_cycle(duty_cycle);
}

void ThreePhasePWM::set_duty_cycle_v(float duty_cycle) {
    phase_v.set_duty_cycle(duty_cycle);
}

void ThreePhasePWM::set_duty_cycle_w(float duty_cycle) {
    phase_w.set_duty_cycle(duty_cycle);
}

void ThreePhasePWM::set_commutation_frequency_hz(uint32_t frequency_hz) {
    phase_u.set_frequency(frequency_hz);
    phase_v.set_frequency(frequency_hz);
    phase_w.set_frequency(frequency_hz);
}

void ThreePhasePWM::set_dead_time_ns(uint32_t dead_time_ns) {
    phase_u.set_dead_time(std::chrono::nanoseconds(dead_time_ns));
    phase_v.set_dead_time(std::chrono::nanoseconds(dead_time_ns));
    phase_w.set_dead_time(std::chrono::nanoseconds(dead_time_ns));
}

MotorDriver::MotorDriver() {}

void MotorDriver::turn_on() {
    for (auto& buffer : buffer_enable) {
        buffer.turn_off();  // buffer enable is active low
    }
    for (auto& reset : gate_driver_reset) {
        reset.turn_on();
    }
    motor_a_pwm.turn_on();
    motor_b_pwm.turn_on();
}

void MotorDriver::turn_off() {
    for (auto& buffer : buffer_enable) {
        buffer.turn_on();  // buffer enable is active low
    }
    for (auto& reset : gate_driver_reset) {
        reset.turn_off();
    }
    motor_a_pwm.turn_off();
    motor_b_pwm.turn_off();
}

void MotorDriver::set_duty_cycle_u(float duty_cycle) {
    motor_a_pwm.set_duty_cycle_u(duty_cycle);
    motor_b_pwm.set_duty_cycle_u(duty_cycle);
}

void MotorDriver::set_duty_cycle_v(float duty_cycle) {
    motor_a_pwm.set_duty_cycle_v(duty_cycle);
    motor_b_pwm.set_duty_cycle_v(duty_cycle);
}

void MotorDriver::set_duty_cycle_w(float duty_cycle) {
    motor_a_pwm.set_duty_cycle_w(duty_cycle);
    motor_b_pwm.set_duty_cycle_w(duty_cycle);
}

void MotorDriver::set_commutation_frequency_hz(uint32_t frequency_hz) {
    motor_a_pwm.set_commutation_frequency_hz(frequency_hz);
    motor_b_pwm.set_commutation_frequency_hz(frequency_hz);
}

void MotorDriver::set_dead_time_ns(uint32_t dead_time_ns) {
    motor_a_pwm.set_dead_time_ns(dead_time_ns);
    motor_b_pwm.set_dead_time_ns(dead_time_ns);
}

};  // namespace BCU::Actuators
