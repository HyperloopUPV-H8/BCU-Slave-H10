#pragma once

#include "BCU/Actuators/MotorDriver.hpp"
#include "ST-LIB.hpp"

namespace BCU::Control {

class Executor {
   public:
    enum ControlMode : uint8_t {
        IDLE = 0,
        TEST_PWM = 1,
    };

   private:
    ControlMode mode{ControlMode::IDLE};

    float duty_cycle_u{0.0f};
    float duty_cycle_v{0.0f};
    float duty_cycle_w{0.0f};

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

    void stop();

    float *get_duty_cycle_u_ptr();
    float *get_duty_cycle_v_ptr();
    float *get_duty_cycle_w_ptr();
};

};  // namespace BCU::Control