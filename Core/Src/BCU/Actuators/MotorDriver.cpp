#include "BCU/Actuators/MotorDriver.hpp"

namespace BCU::Actuators {

ThreePhasePWM::ThreePhasePWM(Pin& phase_u_pin, Pin& phase_u_negated_pin,
                             Pin& phase_v_pin, Pin& phase_v_negated_pin,
                             Pin& phase_w_pin, Pin& phase_w_negated_pin)
    : phase_u{DualPWM{phase_u_pin, phase_u_negated_pin}},
      phase_v{DualPWM{phase_v_pin, phase_v_negated_pin}},
      phase_w{DualPWM{phase_w_pin, phase_w_negated_pin}} {}

void ThreePhasePWM::turn_on() {
    if (locked) return;

    phase_u.turn_on();
    phase_v.turn_on();
    phase_w.turn_on();
}

void ThreePhasePWM::turn_off() {
    if (locked) return;

    phase_u.turn_off();
    phase_v.turn_off();
    phase_w.turn_off();
}

void ThreePhasePWM::turn_off_and_lock() {
    if (locked) return;

    turn_off();
    set_duty_cycle_u(0.0f);
    set_duty_cycle_v(0.0f);
    set_duty_cycle_w(0.0f);
    set_commutation_frequency_hz(0);
    set_dead_time_ns(500);
    locked = true;
}

void ThreePhasePWM::set_duty_cycle_u(float duty_cycle) {
    if (locked) return;

    phase_u.set_duty_cycle(duty_cycle);
}

void ThreePhasePWM::set_duty_cycle_v(float duty_cycle) {
    if (locked) return;

    phase_v.set_duty_cycle(duty_cycle);
}

void ThreePhasePWM::set_duty_cycle_w(float duty_cycle) {
    if (locked) return;

    phase_w.set_duty_cycle(duty_cycle);
}

void ThreePhasePWM::set_commutation_frequency_hz(uint32_t frequency_hz) {
    if (locked) return;

    phase_u.set_frequency(frequency_hz);
    phase_v.set_frequency(frequency_hz);
    phase_w.set_frequency(frequency_hz);
}

void ThreePhasePWM::set_dead_time_ns(uint16_t dead_time_ns) {
    if (locked) return;

    phase_u.set_dead_time(std::chrono::nanoseconds(dead_time_ns));
    phase_v.set_dead_time(std::chrono::nanoseconds(dead_time_ns));
    phase_w.set_dead_time(std::chrono::nanoseconds(dead_time_ns));
}

void MotorDriver::turn_on() {
    for (size_t i{0}; i < 3; ++i) {
        buffer_enable[i].turn_off();  // buffer enable at logic level low
    }
    for (size_t i{0}; i < 4; ++i) {
        gate_driver_reset[i].turn_on();
    }
    motor_a_pwm.turn_on();
    motor_b_pwm.turn_on();
}

void MotorDriver::turn_off() {
    for (size_t i{0}; i < 3; ++i) {
        buffer_enable[i].turn_on();  // buffer disable at logic level high
    }
    for (size_t i{0}; i < 4; ++i) {
        gate_driver_reset[i].turn_off();
    }
    motor_a_pwm.turn_off();
    motor_b_pwm.turn_off();
}

void MotorDriver::turn_off_and_lock() {
    for (size_t i{0}; i < 3; ++i) {
        buffer_enable[i].lock_pin_state(PinState::ON);
    }
    for (size_t i{0}; i < 4; ++i) {
        gate_driver_reset[i].lock_pin_state(PinState::ON);
    }
    motor_a_pwm.turn_off_and_lock();
    motor_b_pwm.turn_off_and_lock();
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

void MotorDriver::set_dead_time_ns(uint16_t dead_time_ns) {
    motor_a_pwm.set_dead_time_ns(dead_time_ns);
    motor_b_pwm.set_dead_time_ns(dead_time_ns);
}

};  // namespace BCU::Actuators
