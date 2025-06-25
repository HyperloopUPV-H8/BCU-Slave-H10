#pragma once

#include "ST-LIB.hpp"

namespace BCU::Sensors {

struct DigitalSensorAdapter {
    PinState value{PinState::OFF};

    DigitalSensor sensor;

    DigitalSensorAdapter(Pin& pin) : sensor(pin, value) {}

    void read() { sensor.read(); }
};

};  // namespace BCU::Sensors
