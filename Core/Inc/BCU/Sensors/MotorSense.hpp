#pragma once

#include "BCU/Pinout.hpp"
#include "BCU/Sensors/DigitalSensorAdapter.hpp"
#include "BCU/Sensors/LinearSensorAdapter.hpp"
#include "ST-LIB.hpp"

namespace BCU::Sensors {

class MotorSense {
    constexpr static double dc_link_voltage_offset{-15.161122213160700};
    constexpr static double dc_link_voltage_slope{138.858744010870000};

    bool fixed_dc_link_voltage{false};
    float dc_link_average_voltage{0.0f};

    LinearSensorAdapter<float> dc_link_voltage[4]{
        LinearSensorAdapter<float>{
            Pinout::dc_link_voltage_1_pin,
            dc_link_voltage_slope,
            dc_link_voltage_offset,
        },
        LinearSensorAdapter<float>{
            Pinout::dc_link_voltage_2_pin,
            dc_link_voltage_slope,
            dc_link_voltage_offset,
        },
        LinearSensorAdapter<float>{
            Pinout::dc_link_voltage_3_pin,
            dc_link_voltage_slope,
            dc_link_voltage_offset,
        },
        LinearSensorAdapter<float>{
            Pinout::dc_link_voltage_4_pin,
            dc_link_voltage_slope,
            dc_link_voltage_offset,
        },
    };

    DigitalSensorAdapter gate_driver_ready[4]{
        Pinout::gate_driver_1_ready_pin,
        Pinout::gate_driver_2_ready_pin,
        Pinout::gate_driver_3_ready_pin,
        Pinout::gate_driver_4_ready_pin,
    };

    DigitalSensorAdapter gate_driver_fault[4]{
        Pinout::gate_driver_1_fault_pin,
        Pinout::gate_driver_2_fault_pin,
        Pinout::gate_driver_3_fault_pin,
        Pinout::gate_driver_4_fault_pin,
    };

   public:
    MotorSense() = default;

    void read_dc_link_voltage();
    void read_gate_driver_ready();
    void read_gate_driver_fault();

    void fix_dc_link_voltage(float voltage);
    void unfix_dc_link_voltage();

    float* get_dc_link_average_voltage_ptr();
    float* get_dc_link_voltage_ptr(size_t index);

    PinState* get_gate_driver_ready_ptr(size_t index);
    PinState* get_gate_driver_fault_ptr(size_t index);
};

}  // namespace BCU::Sensors