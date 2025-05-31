#pragma once

#include "BCU/Actuators/LEDs.hpp"
#include "BCU/Actuators/MotorDriver.hpp"
#include "BCU/Communication/SPI.hpp"
#include "BCU/Control/Executor.hpp"
#include "BCU/Pinout.hpp"
#include "BCU/Sensors/CurrentSense.hpp"
#include "BCU/Sensors/MotorSense.hpp"
#include "BCU/Sensors/PositionSense.hpp"
#include "ST-LIB.hpp"
#include "Shared/StateMachine.hpp"

namespace BCU {

class ProtectionManagerHandle {
   public:
    ProtectionManagerHandle(StateMachine &state_machine,
                            StateMachine::state_id fault_state);
    void start();
    void update_high_frequency();
    void update_low_frequency();
};

class STLIBHandle {
   public:
    STLIBHandle(string mac = "00:80:e1:00:00:00", string ip = "192.168.1.4",
                string subnet_mask = "255.255.0.0",
                string gateway = "192.168.1.1",
                UART::Peripheral &printf_peripheral = UART::uart2);
    void update();
};

class Board {
    using GeneralState = Shared::State::SharedStateMachine::GeneralState;
    using OperationalState = Shared::State::SharedStateMachine::NestedState;

    volatile bool should_update_low_frequency{false};

    Shared::State::SharedStateMachine state_machine;

    ProtectionManagerHandle protection_manager{state_machine.general,
                                               GeneralState::Fault};

    Sensors::MotorSense motor_sensors;
    Sensors::CurrentSense current_sense;
    Sensors::PositionSense position_sense;

    Actuators::MotorDriver motor_driver;
    Actuators::LEDs leds;

    Control::Executor executor{motor_driver};

    Communication::SPI spi{};

    STLIBHandle stlib;

    void initialize_state_machine();
    void initialize_protections();

    void update_connecting();
    void update_operational();
    void update_fault();

    void update_operational_idle();
    void update_operational_precharge();
    void update_operational_ready();
    void update_operational_boosting();
    void update_operational_testing();

   public:
    Board();

    void update();
};

};  // namespace BCU