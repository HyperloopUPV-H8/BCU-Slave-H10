#include "BCU/Sensors/MotorSensors.hpp"

namespace BCU::Sensors {

MotorSensors::MotorSensors(Pin& dc_link_voltage_1_pin,
                           Pin& dc_link_voltage_2_pin,
                           Pin& dc_link_voltage_3_pin,
                           Pin& dc_link_voltage_4_pin)
    : dc_link_voltage{BetterLinearSensor<float>(dc_link_voltage_1_pin,
                                                dc_link_voltage_slope,
                                                dc_link_voltage_offset),
                      BetterLinearSensor<float>(dc_link_voltage_2_pin,
                                                dc_link_voltage_slope,
                                                dc_link_voltage_offset),
                      BetterLinearSensor<float>(dc_link_voltage_3_pin,
                                                dc_link_voltage_slope,
                                                dc_link_voltage_offset),
                      BetterLinearSensor<float>(dc_link_voltage_4_pin,
                                                dc_link_voltage_slope,
                                                dc_link_voltage_offset)} {}

void MotorSensors::read_dc_link_voltage() {
    if (fixed_dc_link_voltage) {
        return;
    }

    dc_link_average_voltage = 0.0f;
    for (auto& sensor : dc_link_voltage) {
        sensor.read();

        dc_link_average_voltage += sensor.value;
    }
    dc_link_average_voltage /= dc_link_voltage.size();
}

void MotorSensors::fix_dc_link_voltage(float voltage) {
    fixed_dc_link_voltage = true;
    dc_link_average_voltage = voltage;
}

void MotorSensors::unfix_dc_link_voltage() { fixed_dc_link_voltage = false; }

float* MotorSensors::get_dc_link_average_voltage_ptr() {
    return &dc_link_average_voltage;
}

float* MotorSensors::get_dc_link_voltage_ptr(size_t index) {
    if (index < dc_link_voltage.size()) {
        return &dc_link_voltage[index].value;
    }
    return nullptr;
}

};  // namespace BCU::Sensors