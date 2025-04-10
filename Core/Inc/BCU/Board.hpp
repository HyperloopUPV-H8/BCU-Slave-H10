#pragma once

#include "BCU/Actuators/LEDs.hpp"
#include "BCU/Actuators/MotorDriver.hpp"
#include "BCU/Communication/SPI.hpp"
#include "BCU/Control/Executor.hpp"
#include "BCU/Pinout.hpp"
#include "BCU/Sensors/MotorDriver.hpp"
#include "BCU/Sensors/PositionEncoder.hpp"
#include "ST-LIB.hpp"
#include "Shared/StateMachine.hpp"

namespace BCU {

// Turn the ProtectionManager into a class
struct ProtectionManagerHandle {
    ProtectionManagerHandle(StateMachine& state_machine) {
        ProtectionManager::initialize();
        ProtectionManager::add_standard_protections();
        ProtectionManager::link_state_machine(
            state_machine,
            Shared::State::SharedStateMachine::GeneralState::Fault);
    }

    void update_low_frequency() { ProtectionManager::check_protections(); }
    void update_high_frequency() {
        ProtectionManager::check_high_frequency_protections();
    }
};

// Turn the STLIB into a class
struct STLIBHandle {
    STLIBHandle(string ip = "192.168.1.4", string subnet_mask = "255.255.0.0",
                string gateway = "192.168.1.1",
                UART::Peripheral& printf_peripheral = UART::uart2) {
        STLIB::start(ip, subnet_mask, gateway, printf_peripheral);
    }

    void update() { STLIB::update(); }
};

class Board {
    Shared::State::SharedStateMachine state_machine;
    ProtectionManagerHandle protection_manager{
        state_machine.general_state_machine};

    Actuators::MotorDriver motor_driver;

    Sensors::PositionEncoder position_encoder;
    Sensors::MotorDriver motor_driver_sensors;

    Control::Executor executor;

    Communication::SPI spi;

    Actuators::LEDs leds;

    STLIBHandle stlib;

    void populate_state_machine();

    void update_connecting();
    void update_operational();
    void update_fault();

    void update_idle();
    void update_ready();
    void update_boosting();
    void update_testing();

   public:
    Board();

    void update();
};

};  // namespace BCU