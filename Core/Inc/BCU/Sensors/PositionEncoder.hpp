#pragma once

#include "ST-LIB.hpp"

namespace BCU::Sensors {

class PositionEncoder {
    struct Speetec {
        using Encoder = EncoderSensor<500>;

        Encoder sensor;
        double position_offset;

        Encoder::Direction direction{Encoder::Direction::FORWARD};
        double position{0.0};
        double velocity{0.0};
        double acceleration{0.0};

        Speetec(Pin& sensor_a, Pin& sensor_b, double position_offset,
                double sample_time_s);

        void turn_on();
        void turn_off();

        void read();
        void reset();

        bool is_detecting_something();
    };

    std::array<Speetec, 3> speetecs;

   public:
    PositionEncoder(std::array<Pin&, 3> sensor_a, std::array<Pin&, 3> sensor_b,
                    std::array<double, 3> position_offset,
                    double sample_time_s);

    void read();

    double get_position();
    double get_velocity();
    double get_acceleration();
};

};  // namespace BCU::Sensors