#include "BCU/Control/VoltageGenerator.hpp"

namespace BCU::Control {

VoltageGenerator::VoltageGenerator(Control::PWMGenerator &pwm_generator,
                                   Sensors::MotorSense &voltage_sense)
    : pwm_generator(pwm_generator), voltage_sense(voltage_sense) {}

void VoltageGenerator::generate_voltage(float voltage_u, float voltage_v,
                                        float voltage_w) {
    float dc_link_voltage{*voltage_sense.get_dc_link_average_voltage_ptr()};

    pwm_generator.generate_duty(voltage_u / dc_link_voltage * 100.0f,
                                voltage_v / dc_link_voltage * 100.0f,
                                voltage_w / dc_link_voltage * 100.0f);
}

};  // namespace BCU::Control