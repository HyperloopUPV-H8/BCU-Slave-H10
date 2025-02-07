#pragma once #pragma once

#include <math.h>

namespace BCU::Control {

class InverseParkTransform {
    constexpr static float PHASE_SHIFT{2.0 * M_PI / 3.0};

    float current_u{0.0};
    float current_v{0.0};
    float current_w{0.0};

    const float &electrical_angle;
    const float &current_active;
    const float &current_reactive;

   public:
    InverseParkTransform(const float &electrical_angle_source,
                         const float &current_active_source,
                         const float &current_reactive_source);

    void execute();

    const float &get_u();
    const float &get_v();
    const float &get_w();
};

}  // namespace BCU::Control
