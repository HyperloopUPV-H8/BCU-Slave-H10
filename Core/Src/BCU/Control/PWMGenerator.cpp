#include "BCU/Control/PWMGenerator.hpp"

namespace BCU::Control {

PWMGenerator::PWMGenerator(Actuators::MotorDriver &driver) : driver(driver) {}

void PWMGenerator::generate_duty(float duty_cycle_u, float duty_cycle_v,
                                 float duty_cycle_w) {
    driver.set_duty_cycle_u(duty_cycle_u > 100.0 ? 100.0
                            : duty_cycle_u < 0.0 ? 0.0
                                                 : duty_cycle_u);
    driver.set_duty_cycle_v(duty_cycle_v > 100.0 ? 100.0
                            : duty_cycle_v < 0.0 ? 0.0
                                                 : duty_cycle_v);
    driver.set_duty_cycle_w(duty_cycle_w > 100.0 ? 100.0
                            : duty_cycle_w < 0.0 ? 0.0
                                                 : duty_cycle_w);
}

};  // namespace BCU::Control
