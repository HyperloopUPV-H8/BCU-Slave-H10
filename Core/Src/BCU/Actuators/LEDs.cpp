#include "BCU/Actuators/LEDs.hpp"

namespace BCU::Actuators {

LEDs::LEDs(Pin &led_operational_pin, Pin &led_fault_pin, Pin &led_can_pin,
           Pin &led_flash_pin, Pin &led_sleep_pin)
    : led_operational(led_operational_pin),
      led_fault(led_fault_pin),
      led_can(led_can_pin),
      led_flash(led_flash_pin),
      led_sleep(led_sleep_pin),
      connecting_blink_action_id(UNDEFINED_ID) {}

void LEDs::signal_connecting() {
    if (connecting_blink_action_id != UNDEFINED_ID) {
        Time::unregister_low_precision_alarm(connecting_blink_action_id);
        connecting_blink_action_id = UNDEFINED_ID;
    }

    connecting_blink_action_id = Time::register_low_precision_alarm(
        CONNECTING_BLINK_PERIOD_MS, [this]() { led_operational.toggle(); });
}

void LEDs::signal_operational() {
    if (connecting_blink_action_id != UNDEFINED_ID) {
        Time::unregister_low_precision_alarm(connecting_blink_action_id);
        connecting_blink_action_id = UNDEFINED_ID;
    }

    led_operational.turn_on();
}

void LEDs::signal_fault() {
    if (connecting_blink_action_id != UNDEFINED_ID) {
        Time::unregister_low_precision_alarm(connecting_blink_action_id);
        connecting_blink_action_id = UNDEFINED_ID;
    }

    led_operational.turn_off();

    led_fault.turn_on();
}

};  // namespace BCU::Actuators
