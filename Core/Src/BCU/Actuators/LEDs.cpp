#include "BCU/Actuators/LEDs.hpp"

namespace BCU::Actuators {

BlinkingLED::BlinkingLED(Pin& pin, uint32_t blink_period_ms)
    : blink_period_ms(blink_period_ms), led(pin) {}

void BlinkingLED::turn_on() {
    stop_blink();
    led.turn_on();
}

void BlinkingLED::turn_off() {
    stop_blink();
    led.turn_off();
}

void BlinkingLED::toggle() {
    stop_blink();
    led.toggle();
}

void BlinkingLED::start_blink() {
    if (is_blinking) return;

    alarm_id = Time::register_low_precision_alarm(blink_period_ms,
                                                  [&]() { led.toggle(); });
    is_blinking = true;
}

void BlinkingLED::stop_blink() {
    if (!is_blinking) return;

    Time::unregister_low_precision_alarm(alarm_id);
    is_blinking = false;
}

void LEDs::signal_operational() { led_operational.turn_on(); }

void LEDs::signal_connecting() { led_operational.start_blink(); }

void LEDs::signal_fault() { led_fault.turn_on(); }

void LEDs::signal_inverter_on() { led_can.turn_on(); }

void LEDs::signal_inverter_off() { led_can.turn_off(); }

void LEDs::turn_all_off() {
    led_operational.turn_off();
    led_fault.turn_off();
    led_can.turn_off();
    led_flash.turn_off();
    led_sleep.turn_off();
}

};  // namespace BCU::Actuators
