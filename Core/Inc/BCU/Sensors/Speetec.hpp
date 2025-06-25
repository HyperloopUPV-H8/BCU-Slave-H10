#pragma once

#include "ST-LIB.hpp"

namespace BCU::Sensors {

template <size_t SAMPLES>
struct Speetec {
    using Sensor = EncoderSensor<SAMPLES>;

    constexpr static double counter_distance_m{0.0001};

    Sensor encoder;

    double position{0.0};
    double speed{0.0};
    double acceleration{0.0};
    Sensor::Direction direction{Sensor::FORWARD};

    Speetec(Pin &pin1, Pin &pin2, double sample_time_s)
        : encoder(pin1, pin2, counter_distance_m, sample_time_s, &direction,
                  &position, &speed, &acceleration) {}

    void turn_on() { encoder.turn_on(); }
    void turn_off() { encoder.turn_off(); }

    void reset() { encoder.reset(); }

    void read() { encoder.read(); }
};

};  // namespace BCU::Sensors
