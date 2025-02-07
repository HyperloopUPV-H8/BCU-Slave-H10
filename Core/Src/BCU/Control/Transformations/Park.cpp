#include "BCU/Control/Transformations/Park.hpp"

using namespace BCU::Control;

void ParkTransform::execute() {
    current_reactive = (current_alpha * sin(electrical_angle)) -
                       (current_beta * cos(electrical_angle));

    current_active = (current_alpha * cos(electrical_angle)) +
                     (current_beta * sin(electrical_angle));
}

ParkTransform::ParkTransform(const float &electrical_angle_source,
                             const float &current_alpha_source,
                             const float &current_beta_source)
    : electrical_angle(electrical_angle_source),
      current_alpha(current_alpha_source),
      current_beta(current_beta_source) {}

const float &ParkTransform::get_active() { return current_active; }
const float &ParkTransform::get_reactive() { return current_reactive; }