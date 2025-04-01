#pragma once

#include "ST-LIB.hpp"

namespace BCU::Sensors {

enum class Direction : uint8_t {
    Forward = 0,
    Backward = 1,
    Undefined = 2,
};

class Speetec {
    double position{0.0};
    bool direction{false};
    double speed{0.0};
    double acceleration{0.0};

    double offset;

    EncoderSensor sensor;

    Direction last_direction{Direction::Undefined};

    Direction get_current_direction() {
        if (last_direction == Direction::Undefined && position == 0.0)
            return Direction::Undefined;

        if (direction)
            return Direction::Forward;
        else
            return Direction::Backward;
    }

   public:
    Speetec(Pin pin1, Pin pin2, double offset_from_start)
        : sensor(pin1, pin2, &position, &direction, &speed, &acceleration),
          offset(offset_from_start) {}

    void start() { sensor.start(); }

    void reset() {
        sensor.reset();
        last_direction = Direction::Undefined;
    }

    void read() { sensor.read(); }

    bool is_detecting_something() const { return speed != 0.0; }

    double get_position() const { return position + offset; }
    double get_speed() const { return speed; }
    double get_acceleration() const { return acceleration; }

    // We want to keep an eye for changes later when we fuse the three sensors
    std::pair<Direction, bool> get_direction() {
        Direction next_direction{get_current_direction()};

        bool has_changed = next_direction != last_direction;
        last_direction = next_direction;
        return {next_direction, has_changed};
    }
};

class BoosterSectionPosition {
    std::array<Speetec, 3> sensors;

    Direction last_direction{Direction::Undefined};
    double last_position{0.0};

   public:
    // Whenever a sensor changes direction, that is the real direction the
    // vehicle is moving, if multiple sensors change direction at the same time,
    // all their direction readings should be the same, if this condition
    // doesn't hold, the result is undefined
    Direction get_direction() {
        Direction next_direction{Direction::Undefined};
        uint8_t changes{0};

        for (Speetec &sensor : sensors) {
            auto [direction, has_changed] = sensor.get_direction();

            if (has_changed) {
                ++changes;
                if (next_direction == Direction::Undefined)
                    next_direction = direction;
                else if (next_direction != direction) {
                    // Incoherence!!
                }
            }
        }

        if (changes > 0) last_direction = next_direction;

        return last_direction;
    }

    double get_position() {
        double position_sum{0.0};
        uint8_t detected_positions{0};

        for (const Speetec &sensor : sensors) {
            if (sensor.is_detecting_something()) {
                ++detected_positions;
                position_sum += sensor.get_position();
            }
        }

        if (detected_positions > 0)
            last_position = position_sum / detected_positions;

        return last_position;
    }

    double get_speed() {
        double speed_sum{0.0};
        uint8_t detected_speeds{0};

        for (const Speetec &sensor : sensors) {
            if (sensor.is_detecting_something()) {
                ++detected_speeds;
                speed_sum += sensor.get_speed();
            }
        }

        if (detected_speeds == 0) return 0.0;

        return speed_sum / detected_speeds;
    }

    double get_acceleration() {
        double acceleration_sum{0.0};
        uint8_t detected_accelerations{0};

        for (const Speetec &sensor : sensors) {
            if (sensor.is_detecting_something()) {
                ++detected_accelerations;
                acceleration_sum += sensor.get_acceleration();
            }
        }

        if (detected_accelerations == 0) return 0.0;

        return acceleration_sum / detected_accelerations;
    }
};

}  // namespace BCU::Sensors