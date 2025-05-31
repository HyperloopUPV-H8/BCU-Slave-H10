#include "BCU/Control/SpaceVector.hpp"

namespace BCU::Control {

SpaceVector::SpaceVector(PWMGenerator &pwm_generator)
    : pwm_generator(pwm_generator) {}

void SpaceVector::update(const float &elapsed_time_s) {
    time_s += elapsed_time_s;

    float sin_u{modulation_index * sin(modulation_frequency_hz * time_s)};
    float sin_v{modulation_index *
                sin(modulation_frequency_hz * time_s + phase_shift)};
    float sin_w{modulation_index *
                sin(modulation_frequency_hz * time_s - phase_shift)};

    float third_harmonic_offset{
        (std::max({sin_u, sin_v, sin_w}) + std::min({sin_u, sin_v, sin_w})) /
        2.0f};

    pwm_generator.generate_duty(
        100.0f * (sin_u - third_harmonic_offset + 1.0f) / 2.0f,
        100.0f * (sin_v - third_harmonic_offset + 1.0f) / 2.0f,
        100.0f * (sin_w - third_harmonic_offset + 1.0f) / 2.0f);
}

void SpaceVector::reset() { time_s = 0.0f; }

float *SpaceVector::get_time_s_ptr() { return &time_s; }

float *SpaceVector::get_modulation_frequency_hz_ptr() {
    return &modulation_frequency_hz;
}

float *SpaceVector::get_modulation_index_ptr() { return &modulation_index; }

void SpaceVector::set_modulation_frequency_hz(const float &frequency_hz) {
    modulation_frequency_hz = frequency_hz;
}

void SpaceVector::set_modulation_index(const float &index) {
    modulation_index = index;
}

};  // namespace BCU::Control
