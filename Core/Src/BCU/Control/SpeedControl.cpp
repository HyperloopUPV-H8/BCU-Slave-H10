#include "BCU/Control/SpeedControl.hpp"

namespace BCU::Control {

void SpeedControl::reset() {
    speed_control.reset();
    target_linear_speed = 0.0;
    measured_linear_speed = 0.0;
    linear_speed_error = 0.0;
}
void SpeedControl::update(float measured_linear_speed) {
    linear_speed_error = measured_linear_speed;
    speed_control.input(linear_speed_error);
    speed_control.execute();

    if (speed_control.output_value > MAX_OUTPUT) {
        saturated_pi_output = MAX_OUTPUT;
    } else if (speed_control.output_value < MIN_OUTPUT) {
        saturated_pi_output = MIN_OUTPUT;
    } else {
        saturated_pi_output = speed_control.output_value;
    }
}
}  // namespace BCU::Control