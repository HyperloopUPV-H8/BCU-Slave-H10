#pragma once

#include <math.h>

namespace BCU::Control {

class ParkTransform {
    float current_d{0.0};
    float current_q{0.0};

    const float &electrical_angle;
    const float &current_alpha;
    const float &current_beta;

   public:
    ParkTransform(const float &electrical_angle_source,
                  const float &current_alpha_source,
                  const float &current_beta_source);

    void execute();

    const float &get_d();
    const float &get_q();
};

}  // namespace BCU::Control
