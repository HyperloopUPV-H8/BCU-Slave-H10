#pragma once

#include "BCU/Control/PWMGenerator.hpp"
#include "BCU/Sensors/MotorSense.hpp"
#include "ST-LIB.hpp"

namespace BCU::Control {

class VoltageGenerator {
    Control::PWMGenerator &pwm_generator;

    Sensors::MotorSense &voltage_sense;

    float voltage_u{0.0f};
    float voltage_v{0.0f};
    float voltage_w{0.0f};

   public:
    VoltageGenerator(Control::PWMGenerator &pwm_generator,
                     Sensors::MotorSense &voltage_sense);

    void generate_voltage(float voltage_u, float voltage_v, float voltage_w);

    float *get_voltage_u_ptr();
    float *get_voltage_v_ptr();
    float *get_voltage_w_ptr();
};

};  // namespace BCU::Control
