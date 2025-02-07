#include "BCU/Control/Transformations/InversePark.hpp"

using namespace BCU::Control;

void InverseParkTransform::execute() {
    current_u = (current_reactive * sin(electrical_angle)) +
                (current_active * cos(electrical_angle));

    current_v = (current_reactive * sin(electrical_angle - PHASE_SHIFT)) +
                (current_active * cos(electrical_angle - PHASE_SHIFT));

    current_w = (current_reactive * sin(electrical_angle + PHASE_SHIFT)) +
                (current_active * cos(electrical_angle + PHASE_SHIFT));
}

InverseParkTransform::InverseParkTransform(const float &electrical_angle_source,
                                           const float &current_active_source,
                                           const float &current_reactive_source)
    : electrical_angle(electrical_angle_source),
      current_active(current_active_source),
      current_reactive(current_reactive_source) {}

const float &InverseParkTransform::get_u() { return current_u; }
const float &InverseParkTransform::get_v() { return current_v; }
const float &InverseParkTransform::get_w() { return current_w; }