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

    static constexpr uint32_t test_pwm_period_us{10000};
    static constexpr uint32_t emulated_movement_period_us{10000};
    static constexpr uint32_t current_control_period_us{10000};

    ModulationMode default_modulation_mode{ModulationMode::THIRD_HARMONIC};

    uint8_t emulated_movement_alarm_id{UNDEFINED_ALARM_ID};
    uint8_t current_control_alarm_id{UNDEFINED_ALARM_ID};
    uint8_t velocity_control_alarm_id{UNDEFINED_ALARM_ID};

    Control::CurrentControl current_control{200, 200, 50, 50,
                                            current_control_period_us / 1e6};

    Control::VelocityControl velocity_control{6, 15,
                                              velocity_control_period_us / 1e6};

    Sensors::MotorDriver &motor_driver_sensors;
    Sensors::PositionEncoder &position_encoder;
    Actuators::MotorDriver &motor_driver;

    double velocity_reference{0.0};
    double velocity_error{0.0};

    double d_current_reference{0.0};
    double d_current_error{0.0};

    double q_current_reference{0.0};
    double q_current_error{0.0};

    double d_target_voltage{0.0};
    double q_target_voltage{0.0};
    double electrical_angle{0.0};

    double u_target_voltage{0.0};
    double v_target_voltage{0.0};
    double w_target_voltage{0.0};

    double u_output_voltage{0.0};
    double v_output_voltage{0.0};
    double w_output_voltage{0.0};

    double u_duty_cycle{0.0};
    double v_duty_cycle{0.0};
    double w_duty_cycle{0.0};

    double three_phase_unbalance{0.0};
    double angular_velocity{0.0};

    ControlMode control_mode{ControlMode::IDLE};
    ModulationMode modulation_mode{ModulationMode::NONE};

    void start_motor_driver();
    void stop_motor_driver();

    void stop_test_pwm();
    void stop_emulated_movement();
    void stop_current_control();
    void stop_velocity_control();

    void read_electrical_angle();
    void modulate_output_voltages();

    void current_control_loop();
    void velocity_control_loop();

   public:
    void use_third_harmonic_modulation();
    void use_sine_modulation();

    void start_test_pwm(double duty_cycle_u, double duty_cycle_v,
                        double duty_cycle_w);
    void set_duty_cycle_u(double duty_cycle);
    void set_duty_cycle_v(double duty_cycle);
    void set_duty_cycle_w(double duty_cycle);

    void start_emulated_movement(double d_current_reference,
                                 double q_current_reference,
                                 double angular_velocity);
    void set_d_current_reference(double current_reference);
    void set_q_current_reference(double current_reference);
    void set_angluar_velocity(double angular_velocity);

    void start_current_control(double d_current_reference,
                               double q_current_reference);

    void start_velocity_control(double velocity_reference);
    void set_velocity_reference(double velocity_reference);

    void stop();
};

};  // namespace BCU::Control