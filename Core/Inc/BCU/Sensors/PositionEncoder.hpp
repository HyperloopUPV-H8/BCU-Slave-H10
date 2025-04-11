#pragma once

#include "ST-LIB.hpp"
#include "Shared/SPI.hpp"

namespace BCU::Sensors {

class PositionEncoder {
    struct Speetec {
        using Encoder = EncoderSensor<500>;

        // Encoder sensor;
        float position_offset;

        Encoder::Direction direction{Encoder::Direction::FORWARD};
        float position{0.0};
        float velocity{0.0};
        float acceleration{0.0};

        Speetec(Pin &sensor_a, Pin &sensor_b, float position_offset,
                float sample_time_s);

        void turn_on();
        void turn_off();

        void read();
        void reset();

        bool is_detecting_something();
    };

    std::array<Speetec, 3> speetecs;

    float average_position{0.0};
    float max_velocity{0.0};
    bool is_detecting{false};

   public:
    PositionEncoder(Pin &speetec_1_1_pin, Pin &speetec_1_2_pin,
                    Pin &speetec_2_1_pin, Pin &speetec_2_2_pin,
                    Pin &speetec_3_1_pin, Pin &speetec_3_2_pin,
                    float position_offset_1, float position_offset_2,
                    float position_offset_3, float sample_time_s);

    void read();
    void reset();
    void turn_on();
    void turn_off();

    float *get_position();
    float *get_velocity();
    bool *is_detecting_something();

    float *get_position_reading(size_t speetec_id);
    float *get_velocity_reading(size_t speetec_id);
    float *get_acceleration_reading(size_t speetec_id);
    Shared::Communication::Direction *get_direction_reading(size_t speetec_id);
};

};  // namespace BCU::Sensors