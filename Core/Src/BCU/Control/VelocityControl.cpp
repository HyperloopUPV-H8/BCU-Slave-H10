#include "BCU/Control/VelocityControl.hpp"

namespace BCU::Control {

VelocityControl::VelocityControl(double velocity_kp, double velocity_ki,
                                 double period)
    : velocity(velocity_kp, velocity_ki, period) {}

void VelocityControl::execute(double velocity_error) {
    velocity.input(velocity_error);

    velocity.execute();

    if (velocity.output_value > VELOCITY_SATURATOR_UPPER_LIMIT) {
        velocity.output_value = VELOCITY_SATURATOR_UPPER_LIMIT;
    } else if (velocity.output_value < VELOCITY_SATURATOR_LOWER_LIMIT) {
        velocity.output_value = VELOCITY_SATURATOR_LOWER_LIMIT;
    }
}

void VelocityControl::reset() { velocity.reset(); }

const double &VelocityControl::get_desired_q_current() const {
    return velocity.output_value;
}

}  // namespace BCU::Control
