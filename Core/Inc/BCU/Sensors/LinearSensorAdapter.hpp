#pragma once

#include "ST-LIB.hpp"

namespace BCU::Sensors {

template <class Type>
    requires std::is_integral_v<Type> || std::is_floating_point_v<Type>
struct LinearSensorAdapter {
    Type value{0.0};

    LinearSensor<Type> sensor;

    LinearSensorAdapter(Pin& pin, Type slope, Type offset)
        : sensor(pin, slope, offset, &value) {}

    void read() { sensor.read(); }
};

};  // namespace BCU::Sensors
