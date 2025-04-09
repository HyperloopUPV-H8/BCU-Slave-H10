#pragma once

#include "ST-LIB.hpp"

namespace BCU::Actuators {

class MotorDriver {
    std::array<DigitalOutput, 3> buffer_enable;

    std::array<DigitalOutput, 4> gate_driver_reset;

    std::array<DualPWM, 4> motor_u;
    std::array<DualPWM, 4> motor_v;
    std::array<DualPWM, 4> motor_w;

   public:
    MotorDriver(std::array<Pin &, 3> buffer_enable_pin,
                std::array<Pin &, 4> gate_driver_reset_pin,
                std::array<Pin &, 4> motor_u_pin,
                std::array<Pin &, 4> motor_u_negated_pin,
                std::array<Pin &, 4> motor_v_pin,
                std::array<Pin &, 4> motor_v_negated_pin,
                std::array<Pin &, 4> motor_w_pin,
                std::array<Pin &, 4> motor_w_negated_pin);

    void enable_buffer();
    void enable_buffer(uint8_t buffer_id);
    void disable_buffer();
    void disable_buffer(uint8_t buffer_id);

    void turn_reset_on();
    void turn_reset_on(uint8_t gate_driver_id);
    void turn_reset_off();
    void turn_reset_off(uint8_t gate_driver_id);

    void turn_on_pwms();
    void turn_on_pwms(uint8_t motor_id);
    void turn_off_pwms();
    void turn_off_pwms(uint8_t motor_id);

    void set_u_duty_cycle(float duty_cycle);
    void set_u_duty_cycle(float duty_cycle, uint8_t motor_id);
    void set_v_duty_cycle(float duty_cycle);
    void set_v_duty_cycle(float duty_cycle, uint8_t motor_id);
    void set_w_duty_cycle(float duty_cycle);
    void set_w_duty_cycle(float duty_cycle, uint8_t motor_id);

    void set_frequency(uint32_t frequency);
    void set_frequency(uint32_t frequency, uint8_t motor_id);

    void set_dead_time(std::chrono::nanoseconds dead_time_ns);
    void set_dead_time(std::chrono::nanoseconds dead_time_ns, uint8_t motor_id);
};

};  // namespace BCU::Actuators