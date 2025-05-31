#pragma once

#include "BCU/Pinout.hpp"
#include "BCU/Sensors/Speetec.hpp"
#include "ST-LIB.hpp"

namespace BCU::Sensors {

class PositionSense {
    constexpr static size_t samples{100};

    using Sensor = EncoderSensor<samples>;

    Speetec<samples> sensors[3]{
        Speetec<samples>{
            Pinout::speetec_1_1_pin,
            Pinout::speetec_1_2_pin,
            0.01,
        },
        Speetec<samples>{
            Pinout::speetec_2_1_pin,
            Pinout::speetec_2_2_pin,
            0.01,
        },
        Speetec<samples>{
            Pinout::speetec_3_1_pin,
            Pinout::speetec_3_2_pin,
            0.01,
        },
    };

    constexpr static double sensor_offset[3]{
        0.0,
        1.0,
        2.0,
    };

    double position{0.0};
    double speed{0.0};
    double acceleration{0.0};
    Sensor::Direction direction{Sensor::FORWARD};

   public:
    PositionSense() = default;

    void turn_on();
    void turn_off();
    void read();

    double *get_position_ptr();
    double *get_speed_ptr();
    double *get_acceleration_ptr();
    Sensor::Direction *get_direction_ptr() {}

    double *get_sensor_position_ptr(size_t index);
    double *get_sensor_speed_ptr(size_t index);
    double *get_sensor_acceleration_ptr(size_t index);
    Sensor::Direction *get_sensor_direction_ptr(size_t index);
};

};  // namespace BCU::Sensors
