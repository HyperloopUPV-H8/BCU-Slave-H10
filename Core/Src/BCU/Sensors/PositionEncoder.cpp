#include "BCU/Sensors/PositionEncoder.hpp"

// Min velocity at which the encoder is considered to be detecting
#define DETECTION_VELOCITY_THRESHOLD 0.00001

namespace BCU::Sensors {

// TODO: Use new speetec implementation
PositionEncoder::Speetec::Speetec(Pin& sensor_a, Pin& sensor_b,
                                  float position_offset, float sample_time_s)
    : sensor(sensor_a, sensor_b, 0.0001, sample_time_s, &direction, &position,
             &velocity, &acceleration),
      position_offset(position_offset) {}

void PositionEncoder::Speetec::turn_on() { sensor.turn_on(); }
void PositionEncoder::Speetec::turn_off() { sensor.turn_off(); }
void PositionEncoder::Speetec::read() { sensor.read(); }
void PositionEncoder::Speetec::reset() { sensor.reset(); }

float* PositionEncoder::Speetec::get_position() { return &position; }
float* PositionEncoder::Speetec::get_velocity() { return &velocity; }
float* PositionEncoder::Speetec::get_acceleration() { return &acceleration; }

bool PositionEncoder::Speetec::is_detecting_something() {
    return velocity >= DETECTION_VELOCITY_THRESHOLD ||
           velocity <= -DETECTION_VELOCITY_THRESHOLD;
}

PositionEncoder::PositionEncoder(Pin& speetec_1_1_pin, Pin& speetec_1_2_pin,
                                 Pin& speetec_2_1_pin, Pin& speetec_2_2_pin,
                                 Pin& speetec_3_1_pin, Pin& speetec_3_2_pin,
                                 float position_offset_1,
                                 float position_offset_2,
                                 float position_offset_3, float sample_time_s)
    : speetecs({Speetec{speetec_1_1_pin, speetec_1_2_pin, position_offset_1,
                        sample_time_s},
                Speetec{speetec_2_1_pin, speetec_2_2_pin, position_offset_2,
                        sample_time_s},
                Speetec{speetec_3_1_pin, speetec_3_2_pin, position_offset_3,
                        sample_time_s}}) {}

void PositionEncoder::read() {
    average_position = 0.0;
    max_velocity = 0.0;

    size_t active_speetecs{0};

    for (auto& speetec : speetecs) {
        speetec.read();
        if (speetec.is_detecting_something()) {
            average_position += speetec.position + speetec.position_offset;
            max_velocity = std::max(max_velocity, speetec.velocity);
            ++active_speetecs;
        }
    }

    average_position /= active_speetecs;
    is_detecting = active_speetecs > 0;
}

void PositionEncoder::reset() {
    for (auto& speetec : speetecs) {
        speetec.reset();
    }
}

void PositionEncoder::turn_on() {
    for (auto& speetec : speetecs) {
        speetec.turn_on();
    }
}

void PositionEncoder::turn_off() {
    for (auto& speetec : speetecs) {
        speetec.turn_off();
    }
}

float* PositionEncoder::get_position() { return &average_position; }

float* PositionEncoder::get_velocity() { return &max_velocity; }

bool* PositionEncoder::is_detecting_something() { return &is_detecting; }

float* PositionEncoder::get_position_reading(size_t speetec_id) {
    return speetecs[speetec_id].get_position();
}

float* PositionEncoder::get_velocity_reading(size_t speetec_id) {
    return speetecs[speetec_id].get_velocity();
}

float* PositionEncoder::get_acceleration_reading(size_t speetec_id) {
    return speetecs[speetec_id].get_acceleration();
}

Shared::Communication::Direction* PositionEncoder::get_direction_reading(
    size_t speetec_id) {
    return (Shared::Communication::Direction*)&(speetecs[speetec_id].direction);
}

};  // namespace BCU::Sensors
