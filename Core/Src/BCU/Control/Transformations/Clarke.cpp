#include "BCU/Control/Transformations/Clarke.hpp"

using namespace BCU::Control;

void ClarkeTransform::execute() {
    current_alpha = ((2.0f * current_u) - current_v - current_w) / 3.0f;

    current_beta =
        ((float(M_SQRT3) * current_v) - (float(M_SQRT3) * current_w)) / 3.0f;

    current_zero = (current_u + current_v + current_w) / 3.0f;
}

ClarkeTransform::ClarkeTransform(const float &current_u_source,
                                 const float &current_v_source,
                                 const float &current_w_source)
    : current_u(current_u_source),
      current_v(current_v_source),
      current_w(current_w_source) {}

const float &ClarkeTransform::get_alpha() { return current_alpha; }
const float &ClarkeTransform::get_beta() { return current_beta; }
const float &ClarkeTransform::get_zero() { return current_zero; }
