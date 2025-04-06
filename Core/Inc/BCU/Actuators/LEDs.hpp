#pragma once

#include "ST-LIB.hpp"

namespace BCU::Actuators {

class LEDs {
    static constexpr uint32_t CONNECTING_BLINK_PERIOD_MS{500};
    static constexpr uint8_t UNDEFINED_ID{std::numeric_limits<uint8_t>::max()};

    DigitalOutput led_operational;
    DigitalOutput led_fault;
    DigitalOutput led_can;
    DigitalOutput led_flash;
    DigitalOutput led_sleep;

    uint8_t connecting_blink_action_id{UNDEFINED_ID};

   public:
    LEDs(Pin &led_operational_pin, Pin &led_fault_pin, Pin &led_can_pin,
         Pin &led_flash_pin, Pin &led_sleep_pin);

    void signal_connecting();

    void signal_operational();

    void signal_fault();
};

};  // namespace BCU::Actuators