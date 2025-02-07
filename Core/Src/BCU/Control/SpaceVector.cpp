#include "BCU/Control/SpaceVector.hpp"

using namespace BCU::Control;

SpaceVectorPWM::SpaceVectorPWM(Pin &u_pin, Pin &u_negated_pin, Pin &v_pin,
                               Pin &v_negated_pin, Pin &w_pin,
                               Pin &w_negated_pin)
    : u_pwm(u_pin, u_negated_pin),
      v_pwm(v_pin, v_negated_pin),
      w_pwm(w_pin, w_negated_pin),
      get_current_u([&]() {
          return modulation_index *
                 sin(TWO_PI * modulation_frequency_hz * time_s);
      }),
      get_current_v([&]() {
          return modulation_index *
                 sin(TWO_PI * modulation_frequency_hz * time_s - PHASE_SHIFT);
      }),
      get_current_w([&]() {
          return modulation_index *
                 sin(TWO_PI * modulation_frequency_hz * time_s + PHASE_SHIFT);
      }) {}

SpaceVectorPWM::SpaceVectorPWM(Pin &u_pin, Pin &u_negated_pin, Pin &v_pin,
                               Pin &v_negated_pin, Pin &w_pin,
                               Pin &w_negated_pin, float &current_u,
                               float &current_v, float &current_w)
    : u_pwm(u_pin, u_negated_pin),
      v_pwm(v_pin, v_negated_pin),
      w_pwm(w_pin, w_negated_pin),
      get_current_u([&]() { return current_u; }),
      get_current_v([&]() { return current_v; }),
      get_current_w([&]() { return current_w; }) {}

void SpaceVectorPWM::update() {
    // execution period is known, if this is not met, we have bigger problems :)
    time_s += execution_period_s;

    // function repeats each modulation period, to prevent overflows reset to
    // zero every TIME_RESET_SCALE periods (not each period to prevent doing too
    // many mod operations)
    if (time_s >= TIME_RESET_SCALE / modulation_frequency_hz)
        time_s = std::fmod(time_s, TIME_RESET_SCALE / modulation_frequency_hz);

    float instantaneous_current_u = get_current_u();
    float instantaneous_current_v = get_current_v();
    float instantaneous_current_w = get_current_w();

    float offset = (std::max({instantaneous_current_u, instantaneous_current_v,
                              instantaneous_current_w}) +
                    std::max({instantaneous_current_u, instantaneous_current_v,
                              instantaneous_current_w})) /
                   2.0f;

    instantaneous_current_u -= offset;
    instantaneous_current_v -= offset;
    instantaneous_current_w -= offset;

    u_pwm.set_duty_cycle((instantaneous_current_u / 2.0f + 0.5f) * 100.0f);
    v_pwm.set_duty_cycle((instantaneous_current_v / 2.0f + 0.5f) * 100.0f);
    w_pwm.set_duty_cycle((instantaneous_current_w / 2.0f + 0.5f) * 100.0f);
}

void SpaceVectorPWM::start() {
    time_s = 0.0f;

    execution_alarm_id = Time::register_mid_precision_alarm(
        execution_period_us, [&]() { update(); });

    u_pwm.turn_on();
    v_pwm.turn_on();
    w_pwm.turn_on();
}

void SpaceVectorPWM::stop() {
    Time::unregister_mid_precision_alarm(execution_alarm_id);
    execution_alarm_id = UNDEFINED_ALARM_ID;

    u_pwm.turn_off();
    v_pwm.turn_off();
    w_pwm.turn_off();

    u_pwm.set_duty_cycle(0.0f);
    v_pwm.set_duty_cycle(0.0f);
    w_pwm.set_duty_cycle(0.0f);
}

void SpaceVectorPWM::set_commutation_frequency(uint32_t freq_hz) {
    u_pwm.set_frequency(freq_hz);
    v_pwm.set_frequency(freq_hz);
    w_pwm.set_frequency(freq_hz);
}

void SpaceVectorPWM::set_commutation_dead_time(
    std::chrono::nanoseconds deadtime) {
    u_pwm.set_dead_time(deadtime);
    v_pwm.set_dead_time(deadtime);
    w_pwm.set_dead_time(deadtime);
}

void SpaceVectorPWM::set_execution_period(uint32_t period_us) {
    execution_period_us = period_us;
    execution_period_s = float(period_us) / 1000000.0f;

    if (execution_alarm_id != UNDEFINED_ALARM_ID) {
        Time::unregister_mid_precision_alarm(execution_alarm_id);
        execution_alarm_id = Time::register_mid_precision_alarm(
            execution_period_us, [&]() { update(); });
    }
}
