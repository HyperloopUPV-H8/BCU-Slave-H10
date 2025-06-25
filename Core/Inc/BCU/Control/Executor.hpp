#pragma once

#include "BCU/Actuators/MotorDriver.hpp"
#include "BCU/Control/CurrentControl.hpp"
#include "BCU/Control/SpeedControl.hpp"
#include "BCU/Sensors/CurrentSense.hpp"
#include "BCU/Sensors/MotorSense.hpp"
#include "BCU/Sensors/PositionSense.hpp"
#include "ST-LIB.hpp"

namespace BCU::Control {

class Executor {
   public:
    constexpr static double control_speed_threshold{0.1};

    enum Mode : uint8_t {
        IDLE = 0,
        TEST_PWM = 1,
        SPACE_VECTOR = 2,
        EMULATED_CURRENT_CONTROL = 3,
        CURRENT_CONTROL = 4,
        SPEED_CONTROL = 5
    };

   private:
    constexpr static uint8_t UNDEFINED_ALARM{UINT8_MAX};

    constexpr static uint32_t space_vector_period_us{200};
    constexpr static uint32_t current_control_period_us{200};
    constexpr static uint32_t speed_control_period_us{1000};

    Mode mode{Mode::IDLE};

    float output_duty_cycle_u{0.0f};
    float output_duty_cycle_v{0.0f};
    float output_duty_cycle_w{0.0f};

    float control_time{0.0f};
    float modulation_index{0.0f};
    float modulation_frequency_hz{0.0f};

    float target_linear_speed{0.0f};
    float linear_speed{0.0};

    double electrical_angle{0.0};
    float measured_u_current{0.0f};
    float measured_v_current{0.0f};
    float measured_w_current{0.0f};
    double measured_alpha_current{0.0};
    double measured_beta_current{0.0};
    double measured_zero_current{0.0};
    double measured_d_current{0.0};
    double measured_q_current{0.0};
    double d_current_ref{0.0};
    double d_current_error{0.0};
    double q_current_ref{0.0};
    double q_current_error{0.0};
    double target_d_voltage{0.0};
    double target_q_voltage{0.0};
    double target_alpha_voltage{0.0};
    double target_beta_voltage{0.0};
    double target_u_voltage{0.0};
    double target_v_voltage{0.0};
    double target_w_voltage{0.0};
    float output_u_voltage{0.0};
    float output_v_voltage{0.0};
    float output_w_voltage{0.0};

    uint8_t control_alarm_id{UNDEFINED_ALARM};

    Sensors::MotorSense &voltage_sense;
    Sensors::CurrentSense &current_sense;
    Sensors::PositionSense &speetec;

    Control::CurrentControl current_controller{current_control_period_us /
                                               1000000.0};
    Control::SpeedControl speed_controller{speed_control_period_us / 1000000.0};
    Actuators::MotorDriver &motor_driver;

    void clarke_transform();
    void park_transform();
    void combined_park_clarke_transformation();
    void inverse_park_transform();
    void inverse_clarke_transform();
    void combined_inverse_park_clarke_transform();
    void update_output_pwms();

    void update_current_control();
    void update_speed_control();

   public:
    Executor(Sensors::MotorSense &voltage_sense,
             Sensors::CurrentSense &current_sense,
             Sensors::PositionSense &speetec,
             Actuators::MotorDriver &motor_driver);

    void start_test_pwm(float duty_cycle_u, float duty_cycle_v,
                        float duty_cycle_w);
    void refresh_test_pwm();
    void set_duty_cycle_u(float duty_cycle_u);
    void set_duty_cycle_v(float duty_cycle_v);
    void set_duty_cycle_w(float duty_cycle_w);

    void start_space_vector(float modulation_index,
                            float modulation_frequency_hz);
    void set_modulation_index(float modulation_index);
    void set_modulation_frequency_hz(float modulation_frequency_hz);

    void start_emulated_current_control(float linear_speed, float d_current,
                                        float q_current);
    void start_current_control(float d_current, float q_current);
    void start_speed_control(float target_linear_speed);
    void set_linear_speed(float linear_speed);
    void set_d_current(float d_current);
    void set_q_current(float q_current);

    void stop();

    float *get_space_vector_time();
    float *get_modulation_index();
    float *get_modulation_frequency_hz();

    double *get_d_current_reference();
    double *get_measured_d_current();
    double *get_d_current_error();
    double *get_q_current_reference();
    double *get_measured_q_current();
    double *get_q_current_error();
    double *get_target_u_voltage();
    double *get_target_v_voltage();
    double *get_target_w_voltage();
    double *get_electrical_angle();
    float *get_target_linear_speed();
    float *get_saturated_pi_output_ptr();
    double *get_saturated_iq_pi_output_ptr() {
        return current_controller.get_q_output_ptr();
    };
    double *get_saturated_id_pi_output_ptr() {
        return current_controller.get_d_output_ptr();
    };
    uint8_t *get_control_mode() { return reinterpret_cast<uint8_t *>(&mode); };
};

};  // namespace BCU::Control