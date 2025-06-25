#include "BCU/Sensors/CurrentSense.hpp"

namespace BCU::Sensors {

void CurrentSense::read() {
    float new_phase_u{0.0f};
    float new_phase_v{0.0f};
    float new_phase_w{0.0f};

    for (size_t i = 0; i < 4; ++i) {
        phase_u[i].read();
        phase_v[i].read();
        phase_w[i].read();

        new_phase_u += phase_u[i].value;
        new_phase_v += phase_v[i].value;
        new_phase_w += phase_w[i].value;
    }

    current_u = new_phase_u / 4.0f;
    current_v = new_phase_v / 4.0f;
    current_w = new_phase_w / 4.0f;
}

float *CurrentSense::get_average_phase_u_current() { return &current_u; }

float *CurrentSense::get_average_phase_v_current() { return &current_v; }

float *CurrentSense::get_average_phase_w_current() { return &current_w; }

float *CurrentSense::get_phase_u_current(size_t index) {
    if (index >= 4) {
        return nullptr;
    }
    return &phase_u[index].value;
}

float *CurrentSense::get_phase_v_current(size_t index) {
    if (index >= 4) {
        return nullptr;
    }
    return &phase_v[index].value;
}

float *CurrentSense::get_phase_w_current(size_t index) {
    if (index >= 4) {
        return nullptr;
    }
    return &phase_w[index].value;
}

};  // namespace BCU::Sensors
