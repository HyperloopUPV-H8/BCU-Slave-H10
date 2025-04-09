#include "BCU/Sensors/PositionEncoder.hpp"

// Min velocity at which the encoder is considered to be detecting
#define DETECTION_VELOCITY_THRESHOLD 0.00001

namespace BCU::Sensors {

// TODO: Use new speetec implementation
PositionEncoder::Speetec::Speetec(Pin& sensor_a, Pin& sensor_b,
                                  double position_offset, double sample_time_s)
    : sensor(sensor_a, sensor_b, 0.0001, sample_time_s, &direction, &position,
             &velocity, &acceleration),
      position_offset(position_offset) {}

void PositionEncoder::Speetec::turn_on() { sensor.turn_on(); }
void PositionEncoder::Speetec::turn_off() { sensor.turn_off(); }
void PositionEncoder::Speetec::read() { sensor.read(); }
void PositionEncoder::Speetec::reset() { sensor.reset(); }

bool PositionEncoder::Speetec::is_detecting_something() {
    return velocity >= DETECTION_VELOCITY_THRESHOLD ||
           velocity <= -DETECTION_VELOCITY_THRESHOLD;
}

PositionEncoder::PositionEncoder(std::array<Pin&, 3> sensor_a,
                                 std::array<Pin&, 3> sensor_b,
                                 std::array<double, 3> position_offset,
                                 double sample_time_s)
    : speetecs{
          Speetec(sensor_a[0], sensor_b[0], position_offset[0], sample_time_s),
          Speetec(sensor_a[1], sensor_b[1], position_offset[1], sample_time_s),
          Speetec(sensor_a[2], sensor_b[2], position_offset[2],
                  sample_time_s)} {}

void PositionEncoder::read() {
    for (auto& speetec : speetecs) {
        speetec.read();
    }
}

// Unless the speetecs are perfectly aligned, the average position will cause a
// small "kick-back" when the next sensor starts reading, this means the
// position will not be exact when going over two different speetecs
double PositionEncoder::get_position() {
    double average_position{0.0};
    size_t active_speetecs{0};

    for (auto& speetec : speetecs) {
        if (!speetec.is_detecting_something()) continue;
        average_position += speetec.position + speetec.position_offset;
        ++active_speetecs;
    }

    return average_position / active_speetecs;
}

// From how we do the position derivative, the max function will prevent the
// "latency" of the derivative from affecting the result that we would get from
// the average, which would show the same kick-back as the possition, only that
// much more prolonged over time, this way the control gets a continuous value
double PositionEncoder::get_velocity() {
    double max_velocity{0.0};
    for (auto& speetec : speetecs) {
        if (!speetec.is_detecting_something()) continue;
        max_velocity = std::max(max_velocity, speetec.velocity);
    }
    return max_velocity;
}

double PositionEncoder::get_acceleration() {
    double total_acceleration{0.0};
    size_t active_speetecs{0};

    for (auto& speetec : speetecs) {
        if (!speetec.is_detecting_something()) continue;
        total_acceleration += speetec.acceleration;
        ++active_speetecs;
    }

    return active_speetecs > 0 ? total_acceleration / active_speetecs : 0.0;
}

};  // namespace BCU::Sensors
