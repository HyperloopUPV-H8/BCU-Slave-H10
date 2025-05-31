#pragma once

#include "BCU/Actuators/MotorDriver.hpp"
#include "ST-LIB.hpp"

namespace BCU::Control {

class PWMGenerator {
    Actuators::MotorDriver &driver;

    float duty_cycle_u{0.0f};
    float duty_cycle_v{0.0f};
    float duty_cycle_w{0.0f};

   public:
    PWMGenerator(Actuators::MotorDriver &driver);

    void generate_duty(float duty_cycle_u, float duty_cycle_v,
                       float duty_cycle_w);

    float *get_duty_cycle_u_ptr();
    float *get_duty_cycle_v_ptr();
    float *get_duty_cycle_w_ptr();
};

};  // namespace BCU::Control
