#pragma once

#include "BCU/Pinout.hpp"
#include "ST-LIB.hpp"

namespace BCU::Actuators {

class BlinkingLED {
    bool is_blinking{false};
    uint8_t alarm_id{0};
    uint32_t blink_period_ms;

    DigitalOutput led;

    void stop_blink();

   public:
    BlinkingLED(Pin& pin, uint32_t blink_period_ms = 500);

    void turn_on();
    void turn_off();
    void toggle();
    void start_blink();
};

class LEDs {
    // Green LED
    BlinkingLED led_operational{Pinout::led_operational_pin};
    // Red LED
    DigitalOutput led_fault{Pinout::led_fault_pin};
    // Blue LED
    DigitalOutput led_can{Pinout::led_can_pin};
    // Yellow LED
    DigitalOutput led_flash{Pinout::led_flash_pin};
    // White LED
    DigitalOutput led_sleep{Pinout::led_sleep_pin};

   public:
    LEDs() = default;

    void signal_operational();

    void signal_connecting();

    void signal_fault();

    void signal_inverter_on();
    void signal_inverter_off();

    void turn_all_off();
};

};  // namespace BCU::Actuators
