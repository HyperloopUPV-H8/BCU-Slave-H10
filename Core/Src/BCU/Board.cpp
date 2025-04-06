#include "BCU/Board.hpp"

namespace BCU {

using namespace Shared::State;

Board::Board()
    : spi(Pinout::spi_ready_slave_pin),
      leds(Pinout::led_operational_pin, Pinout::led_fault_pin,
           Pinout::led_can_pin, Pinout::led_flash_pin, Pinout::led_sleep_pin),
      stlib() {
    populate_state_machine();

    spi.start(&state_machine.general_state_machine.current_state,
              &state_machine.nested_state_machine.current_state);

    Time::register_low_precision_alarm(
        1, [&]() { protection_manager.update_low_frequency(); });
}

void Board::populate_state_machine() {
    state_machine.general_state_machine.add_transition(
        SharedStateMachine::GeneralState::Connecting,
        SharedStateMachine::GeneralState::Operational, [&]() {
            return spi.master_general_state ==
                   SharedStateMachine::GeneralState::Operational;
        });

    state_machine.general_state_machine.add_transition(
        SharedStateMachine::GeneralState::Connecting,
        SharedStateMachine::GeneralState::Fault, [&]() {
            return spi.master_general_state ==
                   SharedStateMachine::GeneralState::Fault;
        });

    state_machine.general_state_machine.add_transition(
        SharedStateMachine::GeneralState::Operational,
        SharedStateMachine::GeneralState::Fault, [&]() {
            return spi.master_general_state ==
                   SharedStateMachine::GeneralState::Fault;
        });

    leds.signal_connecting();

    state_machine.general_state_machine.add_enter_action(
        [&]() { leds.signal_operational(); },
        SharedStateMachine::GeneralState::Operational);

    state_machine.general_state_machine.add_enter_action(
        [&]() { leds.signal_fault(); },
        SharedStateMachine::GeneralState::Fault);
}

void Board::update() {
    state_machine.general_state_machine.check_transitions();
    switch (state_machine.general_state_machine.current_state) {
        case Shared::State::SharedStateMachine::GeneralState::Connecting:
            update_connecting();
            break;
        case Shared::State::SharedStateMachine::GeneralState::Operational:
            update_operational();
            break;
        case Shared::State::SharedStateMachine::GeneralState::Fault:
            update_fault();
            break;
    }

    protection_manager.update_high_frequency();
    spi.update();
    stlib.update();
}

void Board::update_connecting() {}
void Board::update_operational() {
    switch (state_machine.nested_state_machine.current_state) {
        case Shared::State::SharedStateMachine::NestedState::Idle:
            update_idle();
            break;
        case Shared::State::SharedStateMachine::NestedState::Ready:
            update_ready();
            break;
        case Shared::State::SharedStateMachine::NestedState::Boosting:
            update_boosting();
            break;
    }
}
void Board::update_fault() {}

void Board::update_idle() {}
void Board::update_ready() {}
void Board::update_boosting() {}

}  // namespace BCU