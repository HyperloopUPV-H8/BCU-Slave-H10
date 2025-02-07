#pragma once

#include "ST-LIB.hpp"

namespace BCU::Control {

class SpaceVectorPWM {
   public:
    constexpr static uint32_t default_commutation_frequency_hz{50000};
    constexpr static std::chrono::nanoseconds default_deadtime{300};
    constexpr static uint32_t default_modulation_frequency_hz{10};
    constexpr static float default_modulation_index{0.0};
    constexpr static uint32_t default_execution_period_us{200};

   private:
    constexpr static float TWO_PI{2.0 * M_PI};
    constexpr static float PHASE_SHIFT{2.0 * M_PI / 3.0};
    constexpr static float TIME_RESET_SCALE{1000000.0};

    constexpr static uint8_t UNDEFINED_ALARM_ID{UINT8_MAX};

    DualPWM u_pwm;
    DualPWM v_pwm;
    DualPWM w_pwm;

    uint32_t execution_period_us{default_execution_period_us};
    float execution_period_s{float(default_execution_period_us) / 1000000.0f};

    float time_s{0.0f};

    uint8_t execution_alarm_id{UNDEFINED_ALARM_ID};

    std::function<float()> get_current_u;
    std::function<float()> get_current_v;
    std::function<float()> get_current_w;

    void update();

   public:
    float modulation_index{default_modulation_frequency_hz};
    uint32_t modulation_frequency_hz{default_modulation_index};

    SpaceVectorPWM(Pin &u, Pin &u_negated, Pin &v, Pin &v_negated, Pin &w,
                   Pin &w_negated);

    // takes the value for the desired currents from references, instead of
    // calculating the sin
    SpaceVectorPWM(Pin &u, Pin &u_negated, Pin &v, Pin &v_negated, Pin &w,
                   Pin &w_negated, float &current_u, float &current_v,
                   float &current_w);

    void start();
    void stop();

    void set_execution_period(uint32_t period_us);

    void set_commutation_frequency(uint32_t freq_hz);
    void set_commutation_dead_time(std::chrono::nanoseconds deadtime);

    void set_modulation_frequency(uint32_t freq_hz);
};

};  // namespace BCU::Control