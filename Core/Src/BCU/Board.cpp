#include "BCU/Board.hpp"

namespace BCU {

using namespace Shared::State;

Board::Board()
    : motor_driver(Pinout::buffer_1_enable_pin, Pinout::buffer_2_enable_pin,
                   Pinout::buffer_3_enable_pin, Pinout::gate_driver_1_reset_pin,
                   Pinout::gate_driver_2_reset_pin,
                   Pinout::gate_driver_3_reset_pin,
                   Pinout::gate_driver_4_reset_pin, Pinout::phase_u_pwm_a_pin,
                   Pinout::phase_u_pwm_b_pin, Pinout::phase_u_negated_pwm_a_pin,
                   Pinout::phase_u_negated_pwm_b_pin, Pinout::phase_v_pwm_a_pin,
                   Pinout::phase_v_pwm_b_pin, Pinout::phase_v_negated_pwm_a_pin,
                   Pinout::phase_v_negated_pwm_b_pin, Pinout::phase_w_pwm_a_pin,
                   Pinout::phase_w_pwm_b_pin, Pinout::phase_w_negated_pwm_a_pin,
                   Pinout::phase_w_negated_pwm_b_pin),
      executor(motor_driver),
      spi(Pinout::spi_ready_slave_pin,
          &state_machine.general_state_machine.current_state,
          &state_machine.nested_state_machine.current_state),
      stlib() {
    populate_state_machine();

    spi.start();

    motor_driver.set_commutation_frequency_hz(30000);
    motor_driver.set_dead_time_ns(300);

    Time::register_low_precision_alarm(
        1, [&]() { protection_manager.update_low_frequency(); });
}

void Board::populate_state_machine() {
    // ***********
    // Transitions
    // ***********

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

    // ********************
    // Nested Transitions
    // ********************

    state_machine.nested_state_machine.add_transition(
        SharedStateMachine::NestedState::Idle,
        SharedStateMachine::NestedState::Testing,
        [&]() { return spi.has_received_start_test_pwm; });

    state_machine.nested_state_machine.add_transition(
        SharedStateMachine::NestedState::Testing,
        SharedStateMachine::NestedState::Idle,
        [&]() { return spi.has_received_stop_control; });

    // *******
    // Actions
    // *******

    state_machine.general_state_machine.add_enter_action(
        [&]() { executor.stop(); }, SharedStateMachine::GeneralState::Fault);

    // **************
    // Nested Actions
    // **************

    state_machine.nested_state_machine.add_enter_action(
        [&]() {
            executor.stop();

            spi.has_received_start_test_pwm = false;
            spi.has_received_stop_control = false;
        },
        SharedStateMachine::NestedState::Idle);

    state_machine.nested_state_machine.add_enter_action(
        [&]() {
            if (spi.has_received_start_test_pwm) {
                executor.start_test_pwm(spi.requested_duty_cycle_u,
                                        spi.requested_duty_cycle_v,
                                        spi.requested_duty_cycle_w);
            }

            spi.has_received_start_test_pwm = false;
        },
        SharedStateMachine::NestedState::Testing);
}

void Board::update() {
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

    spi.update();
    stlib.update();
    protection_manager.update_high_frequency();
    state_machine.general_state_machine.check_transitions();
}

void Board::update_connecting() {}
void Board::update_operational() {
    if (spi.has_received_configure_commutation_parameters) {
        motor_driver.set_commutation_frequency_hz(
            spi.requested_commutation_frequency_hz);
        motor_driver.set_dead_time_ns(spi.requested_dead_time_ns);

        spi.has_received_configure_commutation_parameters = false;
    }

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
        case Shared::State::SharedStateMachine::NestedState::Testing:
            update_testing();
            break;
    }
}
void Board::update_fault() {}

void Board::update_idle() {
    if (spi.has_received_stop_control) {
        executor.stop();

        spi.has_received_stop_control = false;
    }
}

void Board::update_ready() {}

void Board::update_boosting() {}

void Board::update_testing() {
    if (spi.has_received_start_test_pwm) {
        executor.set_duty_cycle_u(spi.requested_duty_cycle_u);
        executor.set_duty_cycle_v(spi.requested_duty_cycle_v);
        executor.set_duty_cycle_w(spi.requested_duty_cycle_w);

        spi.has_received_start_test_pwm = false;
    }
}

}  // namespace BCU