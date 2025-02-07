#include "BCU/Control/Transformations/InversePark.hpp"

using namespace BCU::Control;

void InverseParkTransform::execute() {
    current_u = (current_d * sin(electrical_angle)) +
                (current_q * cos(electrical_angle));

    current_v = (current_d * sin(electrical_angle - PHASE_SHIFT)) +
                (current_q * cos(electrical_angle - PHASE_SHIFT));

    current_w = (current_d * sin(electrical_angle + PHASE_SHIFT)) +
                (current_q * cos(electrical_angle + PHASE_SHIFT));
}

InverseParkTransform::InverseParkTransform(const float &electrical_angle_source,
                                           const float &current_d_source,
                                           const float &current_q_source)
    : electrical_angle(electrical_angle_source),
      current_d(current_d_source),
      current_q(current_q_source) {}

const float &InverseParkTransform::get_u() { return current_u; }
const float &InverseParkTransform::get_v() { return current_v; }
const float &InverseParkTransform::get_w() { return current_w; }
