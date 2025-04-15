#pragma once

#include "ST-LIB.hpp"

namespace BCU::Sensors {

template <class Type>
    requires std::is_integral_v<Type> || std::is_floating_point_v<Type>
struct BetterLinearSensor {
    Type value{0.0};

    LinearSensor<Type> sensor;

    BetterLinearSensor(Pin& pin, Type slope, Type offset)
        : sensor(pin, slope, offset, &value) {}

    void read() { sensor.read(); }
};

class MotorSensors {
    constexpr static double dc_link_voltage_offset{-15.161122213160700};
    constexpr static double dc_link_voltage_slope{138.858744010870000};

    bool fixed_dc_link_voltage{false};
    float dc_link_average_voltage{0.0f};

    std::array<BetterLinearSensor<float>, 4> dc_link_voltage;

   public:
    MotorSensors(Pin& dc_link_voltage_1_pin, Pin& dc_link_voltage_2_pin,
                 Pin& dc_link_voltage_3_pin, Pin& dc_link_voltage_4_pin);

    void read_dc_link_voltage();

    void fix_dc_link_voltage(float voltage);
    void unfix_dc_link_voltage();

    float* get_dc_link_average_voltage_ptr();
    float* get_dc_link_voltage_ptr(size_t index);
};

}  // namespace BCU::Sensors