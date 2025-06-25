#pragma once

#include "BCU/Pinout.hpp"
#include "BCU/Sensors/LinearSensorAdapter.hpp"
#include "ST-LIB.hpp"

namespace BCU::Sensors {

class CurrentSense {
    constexpr static float current_offset{0.0f};
    constexpr static float current_slope{1.0f};

    LinearSensorAdapter<float> phase_u[4]{
        LinearSensorAdapter<float>{
            Pinout::phase_current_u_1_pin,
            current_slope,
            current_offset,
        },
        LinearSensorAdapter<float>{
            Pinout::phase_current_u_2_pin,
            current_slope,
            current_offset,
        },
        LinearSensorAdapter<float>{
            Pinout::phase_current_u_3_pin,
            current_slope,
            current_offset,
        },
        LinearSensorAdapter<float>{
            Pinout::phase_current_u_4_pin,
            current_slope,
            current_offset,
        },
    };

    LinearSensorAdapter<float> phase_v[4]{
        LinearSensorAdapter<float>{
            Pinout::phase_current_v_1_pin,
            current_slope,
            current_offset,
        },
        LinearSensorAdapter<float>{
            Pinout::phase_current_v_2_pin,
            current_slope,
            current_offset,
        },
        LinearSensorAdapter<float>{
            Pinout::phase_current_v_3_pin,
            current_slope,
            current_offset,
        },
        LinearSensorAdapter<float>{
            Pinout::phase_current_v_4_pin,
            current_slope,
            current_offset,
        },
    };

    LinearSensorAdapter<float> phase_w[4]{
        LinearSensorAdapter<float>{
            Pinout::phase_current_w_1_pin,
            current_slope,
            current_offset,
        },
        LinearSensorAdapter<float>{
            Pinout::phase_current_w_2_pin,
            current_slope,
            current_offset,
        },
        LinearSensorAdapter<float>{
            Pinout::phase_current_w_3_pin,
            current_slope,
            current_offset,
        },
        LinearSensorAdapter<float>{
            Pinout::phase_current_w_4_pin,
            current_slope,
            current_offset,
        },
    };

    float current_u{0.0f};
    float current_v{0.0f};
    float current_w{0.0f};

   public:
    CurrentSense() = default;

    void read();

    float *get_average_phase_u_current();
    float *get_average_phase_v_current();
    float *get_average_phase_w_current();

    float *get_phase_u_current(size_t index);
    float *get_phase_v_current(size_t index);
    float *get_phase_w_current(size_t index);
};

};  // namespace BCU::Sensors
