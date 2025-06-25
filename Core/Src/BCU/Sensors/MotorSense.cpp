#include "BCU/Sensors/MotorSense.hpp"

namespace BCU::Sensors {

void MotorSense::read_dc_link_voltage() {
    if (fixed_dc_link_voltage) {
        return;
    }

    dc_link_average_voltage = 0.0f;
    for (size_t i{0}; i < 4; ++i) {
        dc_link_voltage[i].read();

        dc_link_average_voltage += dc_link_voltage[i].value;
    }
    dc_link_average_voltage /= 4;
}

void MotorSense::read_gate_driver_ready() {
    for (size_t i{0}; i < 4; ++i) {
        gate_driver_ready[i].read();
    }
}

void MotorSense::read_gate_driver_fault() {
    for (size_t i{0}; i < 4; ++i) {
        gate_driver_fault[i].read();
    }
}

void MotorSense::fix_dc_link_voltage(float voltage) {
    fixed_dc_link_voltage = true;
    dc_link_average_voltage = voltage;
}

void MotorSense::unfix_dc_link_voltage() { fixed_dc_link_voltage = false; }

float* MotorSense::get_dc_link_average_voltage_ptr() {
    return &dc_link_average_voltage;
}

float* MotorSense::get_dc_link_voltage_ptr(size_t index) {
    if (index >= 4) {
        return nullptr;
    }
    return &dc_link_voltage[index].value;
}

PinState* MotorSense::get_gate_driver_ready_ptr(size_t index) {
    if (index >= 4) {
        return nullptr;
    }
    return &gate_driver_ready[index].value;
}

PinState* MotorSense::get_gate_driver_fault_ptr(size_t index) {
    if (index >= 4) {
        return nullptr;
    }
    return &gate_driver_fault[index].value;
}

};  // namespace BCU::Sensors