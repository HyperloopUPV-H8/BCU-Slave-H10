#pragma once

#include <math.h>

namespace BCU::Control {

struct ClarkeOutput {
    float alpha;
    float beta;
    float zero;
};

// Takes copies to prevent values from changing mid execution
ClarkeOutput clarke_transform(float current_u, float current_v,
                              float current_w) {
    return {
        .alpha{((2.0f * current_u) - current_v - current_w) / 3.0f},
        .beta{((float(M_SQRT3) * current_v) - (float(M_SQRT3) * current_w)) /
              3.0f},
        .zero{(current_u + current_v + current_w) / 3.0f},
    };
}

}  // namespace BCU::Control
