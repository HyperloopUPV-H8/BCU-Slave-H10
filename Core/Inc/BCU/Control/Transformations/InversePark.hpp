#pragma once #pragma once

#include <math.h>

namespace BCU::Control {

struct InverseParkOutput {
    float u;
    float v;
    float w;
};

// Takes copies to prevent values from changing mid execution
InverseParkOutput inverse_park_transform(float electrical_angle,
                                         float current_d, float current_q) {
    constexpr float PHASE_SHIFT{2.0 * M_PI / 3.0};

    return {
        .u{(current_d * sin(electrical_angle)) +
           (current_q * cos(electrical_angle))},
        .v{(current_d * sin(electrical_angle - PHASE_SHIFT)) +
           (current_q * cos(electrical_angle - PHASE_SHIFT))},
        .w{(current_d * sin(electrical_angle + PHASE_SHIFT)) +
           (current_q * cos(electrical_angle + PHASE_SHIFT))},
    };
}

}  // namespace BCU::Control
