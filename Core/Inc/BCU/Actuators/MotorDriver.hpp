#pragma once

#include "BCU/Pinout.hpp"
#include "ST-LIB.hpp"

namespace BCU::Actuators {

class ThreePhasePWM {
    DualPWM phase_u;
    DualPWM phase_v;
    DualPWM phase_w;

   public:
    ThreePhasePWM(Pin& phase_u_pin, Pin& phase_u_negated_pin, Pin& phase_v_pin,
                  Pin& phase_v_negated_pin, Pin& phase_w_pin,
                  Pin& phase_w_negated_pin);

    void turn_on();
    void turn_off();

    void set_duty_cycle_u(float duty_cycle);
    void set_duty_cycle_v(float duty_cycle);
    void set_duty_cycle_w(float duty_cycle);

    void set_commutation_frequency_hz(uint32_t frequency_hz);
    void set_dead_time_ns(uint16_t dead_time_ns);
};

class MotorDriver {
    std::array<DigitalOutput, 3> buffer_enable{Pinout::buffer_1_enable_pin,
                                               Pinout::buffer_2_enable_pin,
                                               Pinout::buffer_3_enable_pin};

    std::array<DigitalOutput, 4> gate_driver_reset{
        Pinout::gate_driver_1_reset_pin, Pinout::gate_driver_2_reset_pin,
        Pinout::gate_driver_3_reset_pin, Pinout::gate_driver_4_reset_pin};

    ThreePhasePWM motor_a_pwm{
        Pinout::phase_u_pwm_a_pin, Pinout::phase_u_negated_pwm_a_pin,
        Pinout::phase_v_pwm_a_pin, Pinout::phase_v_negated_pwm_a_pin,
        Pinout::phase_w_pwm_a_pin, Pinout::phase_w_negated_pwm_a_pin};
    ThreePhasePWM motor_b_pwm{
        Pinout::phase_u_pwm_b_pin, Pinout::phase_u_negated_pwm_b_pin,
        Pinout::phase_v_pwm_b_pin, Pinout::phase_v_negated_pwm_b_pin,
        Pinout::phase_w_pwm_b_pin, Pinout::phase_w_negated_pwm_b_pin};

   public:
    MotorDriver();

    void turn_on();
    void turn_off();

    void set_duty_cycle_u(float duty_cycle);
    void set_duty_cycle_v(float duty_cycle);
    void set_duty_cycle_w(float duty_cycle);

    void set_commutation_frequency_hz(uint32_t frequency_hz);
    void set_dead_time_ns(uint32_t dead_time_ns);
};

};  // namespace BCU::Actuators