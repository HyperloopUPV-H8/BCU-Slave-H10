#include "BCU/Sensors/PositionSense.hpp"

namespace BCU::Sensors {

void PositionSense::turn_on() {
    for (size_t i{0}; i < 3; ++i) {
        sensors[i].turn_on();
    }
}

void PositionSense::turn_off() {
    for (size_t i{0}; i < 3; ++i) {
        sensors[i].turn_off();
    }
}

void PositionSense::read() {
    double new_position{0.0};
    double new_speed{0.0};
    double new_acceleration{0.0};
    Sensor::Direction new_direction{direction};

    for (size_t i{0}; i < 3; ++i) {
        sensors[i].read();

        new_position =
            std::max(new_position, sensors[i].position + sensor_offset[i]);

        new_speed = std::max(new_speed, sensors[i].speed);

        new_acceleration = std::max(new_acceleration, sensors[i].acceleration);

        if (sensors[i].direction != direction) {
            new_direction = sensors[i].direction;
        }
    }

    position = new_position;
    speed = new_speed;
    acceleration = new_acceleration;
    direction = new_direction;
}

double *PositionSense::get_position_ptr() { return &position; }

double *PositionSense::get_speed_ptr() { return &speed; }

double *PositionSense::get_acceleration_ptr() { return &acceleration; }

PositionSense::Sensor::Direction *PositionSense::get_direction_ptr() {
    return &direction;
}

double *PositionSense::get_sensor_position_ptr(size_t index) {
    if (index >= 3) {
        return nullptr;
    }
    return &sensors[index].position;
}

double *PositionSense::get_sensor_speed_ptr(size_t index) {
    if (index >= 3) {
        return nullptr;
    }
    return &sensors[index].speed;
}

double *PositionSense::get_sensor_acceleration_ptr(size_t index) {
    if (index >= 3) {
        return nullptr;
    }
    return &sensors[index].acceleration;
}

PositionSense::Sensor::Direction *PositionSense::get_sensor_direction_ptr(
    size_t index) {
    if (index >= 3) {
        return nullptr;
    }
    return &sensors[index].direction;
}

};  // namespace BCU::Sensors
