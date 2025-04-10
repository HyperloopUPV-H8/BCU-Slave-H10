#pragma once

#include "ST-LIB.hpp"
#include "Shared/SPI.hpp"

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

        Speetec(Pin &sensor_a, Pin &sensor_b, double position_offset,
                double sample_time_s);

        void turn_on();
        void turn_off();

        void read();
        void reset();

        bool is_detecting_something();
    };

    std::array<Speetec, 3> speetecs;

    double average_position{0.0};
    double max_velocity{0.0};
    bool is_detecting{false};

   public:
    PositionEncoder(std::array<Pin &, 3> sensor_a,
                    std::array<Pin &, 3> sensor_b,
                    std::array<double, 3> position_offset,
                    double sample_time_s);

    void read();
    void reset();
    void turn_on();
    void turn_off();

    double *get_position();
    double *get_velocity();
    bool *is_detecting_something();

    double *get_position_reading(size_t speetec_id);
    double *get_velocity_reading(size_t speetec_id);
    double *get_acceleration_reading(size_t speetec_id);
    Shared::Communication::Direction *get_direction_reading(size_t speetec_id);
};

};  // namespace BCU::Sensors