#pragma once

#include <math.h>

namespace BCU::Control {

struct ParkOutput {
    float d;
    float q;
};

// Takes copies to prevent values from changing mid execution
ParkOutput park_transform(float electrical_angle, float current_alpha,
                          float current_beta) {
    return {
        .d{(current_alpha * sin(electrical_angle)) -
           (current_beta * cos(electrical_angle))},
        .q{(current_alpha * cos(electrical_angle)) +
           (current_beta * sin(electrical_angle))},
    };
}

}  // namespace BCU::Control
