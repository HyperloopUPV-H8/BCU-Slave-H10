#pragma once

#include "BCU/Actuators/MotorDriver.hpp"
#include "ST-LIB.hpp"

namespace BCU::Control {

class Executor {
   public:
    enum ControlMode : uint8_t { IDLE = 0, TEST_PWM = 1, SPACE_VECTOR = 2 };

   private:
    static constexpr uint32_t space_vector_period_us{200};

    ControlMode mode{ControlMode::IDLE};

    uint8_t space_vector_alarm_id{0};

    float duty_cycle_u{0.0f};
    float duty_cycle_v{0.0f};
    float duty_cycle_w{0.0f};

    float modulation_index{0.0f};
    float modulation_frequency_hz{0.0f};
    float space_vector_time{0.0f};

    Actuators::MotorDriver &motor_driver;

    void start_motor_driver();
    void stop_motor_driver();

    void update_duty_cycle();

   public:
    Executor(Actuators::MotorDriver &motor_driver);

    void start_test_pwm(float duty_cycle_u, float duty_cycle_v,
                        float duty_cycle_w);
    void set_duty_cycle_u(float duty_cycle);
    void set_duty_cycle_v(float duty_cycle);
    void set_duty_cycle_w(float duty_cycle);

    void start_space_vector(float modulation_index,
                            float modulation_frequency_hz);
    void set_modulation_index(float modulation_index);
    void set_modulation_frequency_hz(float modulation_frequency_hz);

    void configure_current_control(float current_d_ref, float current_q_ref);
    void configure_speed_control(float speed_ref);

    void start_selected_control();

    void stop();

    float *get_duty_cycle_u_ptr();
    float *get_duty_cycle_v_ptr();
    float *get_duty_cycle_w_ptr();
};

};  // namespace BCU::Control