#pragma once

#include <math.h>

namespace BCU::Control {

class ClarkeTransform {
    float current_alpha{0.0};
    float current_beta{0.0};
    float current_zero{0.0};

    const float &current_u;
    const float &current_v;
    const float &current_w;

   public:
    ClarkeTransform(const float &current_u_source,
                    const float &current_v_source,
                    const float &current_w_source);

    void execute();

    const float &get_alpha();
    const float &get_beta();
    const float &get_zero();
};

}  // namespace BCU::Control
