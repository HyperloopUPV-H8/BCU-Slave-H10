#pragma once

#include "BCU/Actuators/MotorDriver.hpp"
#include "BCU/Control/CurrentControl.hpp"
#include "BCU/Control/Transformations.hpp"
#include "BCU/Control/VelocityControl.hpp"
#include "BCU/Sensors/MotorDriver.hpp"
#include "BCU/Sensors/PositionEncoder.hpp"
#include "ST-LIB.hpp"

namespace BCU::Control {

class Executor {
   public:
    enum ControlMode : uint8_t {
        IDLE = 0,
        TEST_PWM = 1,
        EMULATED_MOVEMENT = 2,
        CURRENT_CONTROL = 3,
        VELOCITY_CONTROL = 4,
    };

    enum ModulationMode : uint8_t {
        NONE = 0,
        SINE = 1,
        THIRD_HARMONIC = 2,
    };

   private:
    static constexpr uint8_t UNDEFINED_ALARM_ID{UINT8_MAX};

    static constexpr uint32_t emulated_movement_period_us{200};
    static constexpr uint32_t current_control_period_us{200};
    static constexpr uint32_t velocity_control_period_us{200};

    ModulationMode default_modulation_mode{ModulationMode::THIRD_HARMONIC};

    uint8_t emulated_movement_alarm_id{UNDEFINED_ALARM_ID};
    uint8_t current_control_alarm_id{UNDEFINED_ALARM_ID};
    uint8_t velocity_control_alarm_id{UNDEFINED_ALARM_ID};

    Control::CurrentControl current_control{200, 200, 50, 50,
                                            current_control_period_us / 1e6};

    Control::VelocityControl velocity_control{6, 15,
                                              velocity_control_period_us / 1e6};

    Actuators::MotorDriver &motor_driver;
    Sensors::MotorDriver &motor_driver_sensors;
    Sensors::PositionEncoder &position_encoder;

    float velocity_reference{0.0};
    float velocity_measurement{0.0};
    float velocity_error{0.0};

    float u_current_measurement{0.0};
    float v_current_measurement{0.0};
    float w_current_measurement{0.0};
    float electrical_angle{0.0};

    float d_current_reference{0.0};
    float d_current_measurement{0.0};
    float d_current_error{0.0};

    float q_current_reference{0.0};
    float q_current_measurement{0.0};
    float q_current_error{0.0};

    float three_phase_unbalance{0.0};

    float d_target_voltage{0.0};
    float q_target_voltage{0.0};

    float u_target_voltage{0.0};
    float v_target_voltage{0.0};
    float w_target_voltage{0.0};

    float u_output_voltage{0.0};
    float v_output_voltage{0.0};
    float w_output_voltage{0.0};

    float u_duty_cycle{0.0};
    float v_duty_cycle{0.0};
    float w_duty_cycle{0.0};

    float angular_velocity{0.0};

    ControlMode control_mode{ControlMode::IDLE};
    ModulationMode modulation_mode{ModulationMode::NONE};

    void start_motor_driver();
    void stop_motor_driver();

    void current_control_loop();
    void velocity_control_loop();

    void read_electrical_angle();
    void modulate_output_voltages();

    void stop_test_pwm();
    void stop_emulated_movement();
    void stop_current_control();
    void stop_velocity_control();

   public:
    Executor(Actuators::MotorDriver &motor_driver,
             Sensors::MotorDriver &motor_driver_sensors,
             Sensors::PositionEncoder &position_encoder);

    void start_velocity_control(float velocity_reference);

    void start_current_control(float d_current_reference,
                               float q_current_reference);

    void start_emulated_movement(float d_current_reference,
                                 float q_current_reference,
                                 float angular_velocity);

    void start_test_pwm(float duty_cycle_u, float duty_cycle_v,
                        float duty_cycle_w);

    void use_third_harmonic_modulation();
    void use_sine_modulation();

    void set_velocity_reference(float velocity_reference);

    void set_d_current_reference(float current_reference);
    void set_q_current_reference(float current_reference);

    void set_angluar_velocity(float angular_velocity);

    void set_duty_cycle_u(float duty_cycle);
    void set_duty_cycle_v(float duty_cycle);
    void set_duty_cycle_w(float duty_cycle);

    void stop();

    float *get_velocity_reference();
    float *get_velocity_error();
    float *get_u_current_measurement();
    float *get_v_current_measurement();
    float *get_w_current_measurement();
    float *get_electrical_angle();
    float *get_d_current_reference();
    float *get_d_current_measurement();
    float *get_d_current_error();
    float *get_q_current_reference();
    float *get_q_current_measurement();
    float *get_q_current_error();
    float *get_three_phase_unbalance();
    float *get_d_target_voltage();
    float *get_q_target_voltage();
    float *get_u_target_voltage();
    float *get_v_target_voltage();
    float *get_w_target_voltage();
    float *get_u_output_voltage();
    float *get_v_output_voltage();
    float *get_w_output_voltage();
    float *get_u_duty_cycle();
    float *get_v_duty_cycle();
    float *get_w_duty_cycle();
    float *get_angular_velocity();
};

};  // namespace BCU::Control