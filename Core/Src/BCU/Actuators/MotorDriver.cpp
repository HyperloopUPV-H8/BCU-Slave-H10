#include "BCU/Actuators/MotorDriver.hpp"

namespace BCU::Actuators {

MotorDriver::MotorDriver(Pin& buffer_enable_pin_1, Pin& buffer_enable_pin_2,
                         Pin& buffer_enable_pin_3, Pin& gate_driver_reset_pin_1,
                         Pin& gate_driver_reset_pin_2,
                         Pin& gate_driver_reset_pin_3,
                         Pin& gate_driver_reset_pin_4, Pin& phase_u_pwm_a_pin,
                         Pin& phase_u_pwm_b_pin, Pin& phase_u_negated_pwm_a_pin,
                         Pin& phase_u_negated_pwm_b_pin, Pin& phase_v_pwm_a_pin,
                         Pin& phase_v_pwm_b_pin, Pin& phase_v_negated_pwm_a_pin,
                         Pin& phase_v_negated_pwm_b_pin, Pin& phase_w_pwm_a_pin,
                         Pin& phase_w_pwm_b_pin, Pin& phase_w_negated_pwm_a_pin,
                         Pin& phase_w_negated_pwm_b_pin)
    : buffer_enable({DigitalOutput{buffer_enable_pin_1},
                     DigitalOutput{buffer_enable_pin_2},
                     DigitalOutput{buffer_enable_pin_3}}),
      gate_driver_reset({DigitalOutput{gate_driver_reset_pin_1},
                         DigitalOutput{gate_driver_reset_pin_2},
                         DigitalOutput{gate_driver_reset_pin_3},
                         DigitalOutput{gate_driver_reset_pin_4}}),
      motor_u({DualPWM{phase_u_pwm_a_pin, phase_u_negated_pwm_a_pin},
               DualPWM{phase_u_pwm_b_pin, phase_u_negated_pwm_b_pin}}),
      motor_v({DualPWM{phase_v_pwm_a_pin, phase_v_negated_pwm_a_pin},
               DualPWM{phase_v_pwm_b_pin, phase_v_negated_pwm_b_pin}}),
      motor_w({DualPWM{phase_w_pwm_a_pin, phase_w_negated_pwm_a_pin},
               DualPWM{phase_w_pwm_b_pin, phase_w_negated_pwm_b_pin}}) {}

void MotorDriver::enable_buffer() {
    for (uint8_t i{0}; i < buffer_enable.size(); ++i) {
        enable_buffer(i);
    }
}

void MotorDriver::enable_buffer(uint8_t buffer_id) {
    buffer_enable[buffer_id].turn_off();
}

void MotorDriver::disable_buffer() {
    for (uint8_t i{0}; i < buffer_enable.size(); ++i) {
        disable_buffer(i);
    }
}

void MotorDriver::disable_buffer(uint8_t buffer_id) {
    buffer_enable[buffer_id].turn_on();
}

void MotorDriver::turn_reset_on() {
    for (uint8_t i{0}; i < gate_driver_reset.size(); ++i) {
        turn_reset_on(i);
    }
}

void MotorDriver::turn_reset_on(uint8_t gate_driver_id) {
    gate_driver_reset[gate_driver_id].turn_on();
}

void MotorDriver::turn_reset_off() {
    for (uint8_t i{0}; i < gate_driver_reset.size(); ++i) {
        turn_reset_off(i);
    }
}

void MotorDriver::turn_reset_off(uint8_t gate_driver_id) {
    gate_driver_reset[gate_driver_id].turn_off();
}

void MotorDriver::turn_on_pwms() {
    for (uint8_t i{0}; i < motor_u.size(); ++i) {
        turn_on_pwms(i);
    }
}

void MotorDriver::turn_on_pwms(uint8_t motor_id) {
    motor_u[motor_id].turn_on();
    motor_v[motor_id].turn_on();
    motor_w[motor_id].turn_on();
}

void MotorDriver::turn_off_pwms() {
    for (uint8_t i{0}; i < motor_u.size(); ++i) {
        turn_off_pwms(i);
    }
}

void MotorDriver::turn_off_pwms(uint8_t motor_id) {
    motor_u[motor_id].turn_off();
    motor_v[motor_id].turn_off();
    motor_w[motor_id].turn_off();
}

void MotorDriver::set_u_duty_cycle(float duty_cycle) {
    for (uint8_t i{0}; i < motor_u.size(); ++i) {
        set_u_duty_cycle(duty_cycle, i);
    }
}

void MotorDriver::set_u_duty_cycle(float duty_cycle, uint8_t motor_id) {
    motor_u[motor_id].set_duty_cycle(duty_cycle);
}

void MotorDriver::set_v_duty_cycle(float duty_cycle) {
    for (uint8_t i{0}; i < motor_v.size(); ++i) {
        set_v_duty_cycle(duty_cycle, i);
    }
}

void MotorDriver::set_v_duty_cycle(float duty_cycle, uint8_t motor_id) {
    motor_v[motor_id].set_duty_cycle(duty_cycle);
}

void MotorDriver::set_w_duty_cycle(float duty_cycle) {
    for (uint8_t i{0}; i < motor_w.size(); ++i) {
        set_w_duty_cycle(duty_cycle, i);
    }
}

void MotorDriver::set_w_duty_cycle(float duty_cycle, uint8_t motor_id) {
    motor_w[motor_id].set_duty_cycle(duty_cycle);
}

void MotorDriver::set_frequency(uint32_t frequency) {
    for (uint8_t i{0}; i < motor_u.size(); ++i) {
        set_frequency(frequency, i);
    }
}

void MotorDriver::set_frequency(uint32_t frequency, uint8_t motor_id) {
    motor_u[motor_id].set_frequency(frequency);
    motor_v[motor_id].set_frequency(frequency);
    motor_w[motor_id].set_frequency(frequency);
}

void MotorDriver::set_dead_time(std::chrono::nanoseconds dead_time_ns) {
    for (uint8_t i{0}; i < motor_u.size(); ++i) {
        set_dead_time(dead_time_ns, i);
    }
}

void MotorDriver::set_dead_time(std::chrono::nanoseconds dead_time_ns,
                                uint8_t motor_id) {
    motor_u[motor_id].set_dead_time(dead_time_ns);
    motor_v[motor_id].set_dead_time(dead_time_ns);
    motor_w[motor_id].set_dead_time(dead_time_ns);
}

};  // namespace BCU::Actuators
