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

    constexpr static double MIN_SPEED_FOR_BOOSTER{0.1};

    volatile bool should_update_1khz{false};
    volatile bool should_update_60hz{false};
    volatile bool should_update_encoder{false};
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

    Communication::SPI spi{
        &state_machine.general.current_state,
        &state_machine.nested.current_state,
        position_sense.get_position_ptr(),
        position_sense.get_speed_ptr(),
        position_sense.get_acceleration_ptr(),
        (uint8_t *)position_sense.get_direction_ptr(),
        position_sense.get_sensor_position_ptr(0),
        position_sense.get_sensor_speed_ptr(0),
        position_sense.get_sensor_acceleration_ptr(0),
        (uint8_t *)position_sense.get_sensor_direction_ptr(0),
        position_sense.get_sensor_position_ptr(1),
        position_sense.get_sensor_speed_ptr(1),
        position_sense.get_sensor_acceleration_ptr(1),
        (uint8_t *)position_sense.get_sensor_direction_ptr(1)};

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