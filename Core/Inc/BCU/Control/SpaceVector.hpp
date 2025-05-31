#pragma once

#include "BCU/Control/PWMGenerator.hpp"
#include "ST-LIB.hpp"

namespace BCU::Control {

class SpaceVector {
    constexpr static float phase_shift{2.0f * M_PI / 3.0f};

    PWMGenerator &pwm_generator;

    float time_s{0.0f};

    float modulation_frequency_hz{0.0f};
    float modulation_index{0.0f};

   public:
    SpaceVector(PWMGenerator &pwm_generator);

    void update(const float &elapsed_time_s);

    void reset();

    float *get_time_s_ptr();
    float *get_modulation_frequency_hz_ptr();
    float *get_modulation_index_ptr();

    void set_modulation_frequency_hz(const float &frequency_hz);
    void set_modulation_index(const float &index);
};

};  // namespace BCU::Control
