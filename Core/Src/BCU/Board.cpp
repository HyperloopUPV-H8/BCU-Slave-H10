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
      position_encoder(Pinout::speetec_1_1_pin, Pinout::speetec_1_2_pin,
                       Pinout::speetec_2_1_pin, Pinout::speetec_2_2_pin,
                       Pinout::speetec_3_1_pin, Pinout::speetec_3_2_pin, 0.0,
                       1.5, 3.0, 0.001),
      motor_driver_sensors(
          Pinout::dc_link_voltage_1_pin, Pinout::dc_link_voltage_2_pin,
          Pinout::dc_link_voltage_3_pin, Pinout::dc_link_voltage_4_pin,
          Pinout::phase_current_u_1_pin, Pinout::phase_current_v_1_pin,
          Pinout::phase_current_w_1_pin, Pinout::phase_current_u_2_pin,
          Pinout::phase_current_v_2_pin, Pinout::phase_current_w_2_pin,
          Pinout::phase_current_u_3_pin, Pinout::phase_current_v_3_pin,
          Pinout::phase_current_w_3_pin, Pinout::phase_current_u_4_pin,
          Pinout::phase_current_v_4_pin, Pinout::phase_current_w_4_pin,
          Pinout::gate_driver_1_fault_pin, Pinout::gate_driver_2_fault_pin,
          Pinout::gate_driver_3_fault_pin, Pinout::gate_driver_4_fault_pin,
          Pinout::gate_driver_1_ready_pin, Pinout::gate_driver_2_ready_pin,
          Pinout::gate_driver_3_ready_pin, Pinout::gate_driver_4_ready_pin),
      executor(motor_driver, motor_driver_sensors, position_encoder),
      spi(Pinout::spi_ready_slave_pin,
          &state_machine.general_state_machine.current_state,
          &state_machine.nested_state_machine.current_state,
          executor.get_velocity_reference(), executor.get_velocity_error(),
          executor.get_u_current_measurement(),
          executor.get_v_current_measurement(),
          executor.get_w_current_measurement(), executor.get_electrical_angle(),
          executor.get_d_current_reference(),
          executor.get_d_current_measurement(), executor.get_d_current_error(),
          executor.get_q_current_reference(),
          executor.get_q_current_measurement(), executor.get_q_current_error(),
          executor.get_three_phase_unbalance(), executor.get_d_target_voltage(),
          executor.get_q_target_voltage(), executor.get_u_target_voltage(),
          executor.get_v_target_voltage(), executor.get_w_target_voltage(),
          executor.get_u_output_voltage(), executor.get_v_output_voltage(),
          executor.get_w_output_voltage(), executor.get_u_duty_cycle(),
          executor.get_v_duty_cycle(), executor.get_w_duty_cycle(),
          executor.get_angular_velocity(),
          {position_encoder.get_position_reading(1),
           position_encoder.get_position_reading(2),
           position_encoder.get_position_reading(3)},
          {position_encoder.get_velocity_reading(1),
           position_encoder.get_velocity_reading(2),
           position_encoder.get_velocity_reading(3)},
          {position_encoder.get_acceleration_reading(1),
           position_encoder.get_acceleration_reading(2),
           position_encoder.get_acceleration_reading(3)},
          {position_encoder.get_direction_reading(1),
           position_encoder.get_direction_reading(2),
           position_encoder.get_direction_reading(3)},
          position_encoder.get_position(), position_encoder.get_velocity(),
          position_encoder.is_detecting_something(),
          {motor_driver_sensors.get_dc_link_voltage_ptr(0),
           motor_driver_sensors.get_dc_link_voltage_ptr(1),
           motor_driver_sensors.get_dc_link_voltage_ptr(2),
           motor_driver_sensors.get_dc_link_voltage_ptr(3)},
          {std::array<float *, 3>{
               motor_driver_sensors.get_motor_phase_u_current_ptr(0),
               motor_driver_sensors.get_motor_phase_v_current_ptr(0),
               motor_driver_sensors.get_motor_phase_w_current_ptr(0)},
           std::array<float *, 3>{
               motor_driver_sensors.get_motor_phase_u_current_ptr(1),
               motor_driver_sensors.get_motor_phase_v_current_ptr(1),
               motor_driver_sensors.get_motor_phase_w_current_ptr(1)},
           std::array<float *, 3>{
               motor_driver_sensors.get_motor_phase_u_current_ptr(2),
               motor_driver_sensors.get_motor_phase_v_current_ptr(2),
               motor_driver_sensors.get_motor_phase_w_current_ptr(2)},
           std::array<float *, 3>{
               motor_driver_sensors.get_motor_phase_u_current_ptr(3),
               motor_driver_sensors.get_motor_phase_v_current_ptr(3),
               motor_driver_sensors.get_motor_phase_w_current_ptr(3)}},
          {motor_driver_sensors.get_gate_driver_fault_ptr(0),
           motor_driver_sensors.get_gate_driver_fault_ptr(1),
           motor_driver_sensors.get_gate_driver_fault_ptr(2),
           motor_driver_sensors.get_gate_driver_fault_ptr(3)},
          {motor_driver_sensors.get_gate_driver_ready_ptr(0),
           motor_driver_sensors.get_gate_driver_ready_ptr(1),
           motor_driver_sensors.get_gate_driver_ready_ptr(2),
           motor_driver_sensors.get_gate_driver_ready_ptr(3)}),
      leds(Pinout::led_operational_pin, Pinout::led_fault_pin,
           Pinout::led_can_pin, Pinout::led_flash_pin, Pinout::led_sleep_pin),
      stlib() {
    populate_state_machine();
    leds.signal_connecting();

    spi.start();

    motor_driver.set_frequency(30000);
    motor_driver.set_dead_time(std::chrono::nanoseconds(300));

    Time::register_low_precision_alarm(
        1, [&]() { protection_manager.update_low_frequency(); });

    position_encoder.turn_on();
    Time::register_mid_precision_alarm(200, [&]() {
        motor_driver_sensors.read_motor_phase_current();
        position_encoder.read();
    });

    Time::register_low_precision_alarm(100, [&]() {
        motor_driver_sensors.read_dc_link_voltage();
        motor_driver_sensors.read_gate_driver_fault();
        motor_driver_sensors.read_gate_driver_ready();
    });
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

    // ********************
    // Nested Transitions
    // ********************

    state_machine.nested_state_machine.add_transition(
        SharedStateMachine::NestedState::Idle,
        SharedStateMachine::NestedState::Ready,
        [&]() { return spi.enable_booster; });

    state_machine.nested_state_machine.add_transition(
        SharedStateMachine::NestedState::Idle,
        SharedStateMachine::NestedState::Testing, [&]() {
            return spi.has_received_start_emulated_movement_order ||
                   spi.has_received_start_test_pwm_order;
        });

    state_machine.nested_state_machine.add_transition(
        SharedStateMachine::NestedState::Ready,
        SharedStateMachine::NestedState::Boosting, [&]() {
            return spi.enable_booster &&
                   (spi.has_received_start_current_control_order ||
                    spi.has_received_start_velocity_control_order);
        });

    state_machine.nested_state_machine.add_transition(
        SharedStateMachine::NestedState::Ready,
        SharedStateMachine::NestedState::Idle,
        [&]() { return !spi.enable_booster; });

    state_machine.nested_state_machine.add_transition(
        SharedStateMachine::NestedState::Boosting,
        SharedStateMachine::NestedState::Idle, [&]() {
            return !(spi.enable_booster &&
                     position_encoder.is_detecting_something());
        });

    state_machine.nested_state_machine.add_transition(
        SharedStateMachine::NestedState::Testing,
        SharedStateMachine::NestedState::Idle,
        [&]() { return spi.has_received_stop_control_order; });

    // *******
    // Actions
    // *******

    state_machine.general_state_machine.add_enter_action(
        [&]() { leds.signal_operational(); },
        SharedStateMachine::GeneralState::Operational);

    state_machine.general_state_machine.add_enter_action(
        [&]() {
            executor.stop();
            leds.signal_fault();
        },
        SharedStateMachine::GeneralState::Fault);

    // **************
    // Nested Actions
    // **************

    state_machine.nested_state_machine.add_enter_action(
        [&]() {
            position_encoder.reset();
            executor.stop();

            spi.has_received_start_current_control_order = false;
            spi.has_received_start_velocity_control_order = false;
            spi.has_received_start_emulated_movement_order = false;
            spi.has_received_start_test_pwm_order = false;
            spi.has_received_stop_control_order = false;
            spi.enable_booster = false;
        },
        SharedStateMachine::NestedState::Idle);

    state_machine.nested_state_machine.add_enter_action(
        [&]() {
            leds.signal_boosting();

            if (spi.has_received_start_velocity_control_order) {
                executor.start_velocity_control(
                    spi.requested_velocity_reference);

            } else if (spi.has_received_start_current_control_order) {
                executor.start_current_control(
                    spi.requested_d_current_reference,
                    spi.requested_q_current_reference);
            }

            spi.has_received_start_velocity_control_order = false;
            spi.has_received_start_current_control_order = false;
        },
        SharedStateMachine::NestedState::Boosting);

    state_machine.nested_state_machine.add_exit_action(
        [&]() {
            executor.stop();
            leds.signal_not_boosting();
        },
        SharedStateMachine::NestedState::Boosting);

    state_machine.nested_state_machine.add_enter_action(
        [&]() {
            leds.signal_boosting();

            if (spi.has_received_start_emulated_movement_order) {
                executor.start_emulated_movement(
                    spi.requested_d_current_reference,
                    spi.requested_q_current_reference,
                    spi.requested_angular_velocity);
            } else if (spi.has_received_start_test_pwm_order) {
                executor.start_test_pwm(spi.requested_duty_cycle_u,
                                        spi.requested_duty_cycle_v,
                                        spi.requested_duty_cycle_w);
            }

            spi.has_received_start_emulated_movement_order = false;
            spi.has_received_start_test_pwm_order = false;
        },
        SharedStateMachine::NestedState::Testing);

    state_machine.nested_state_machine.add_exit_action(
        [&]() {
            executor.stop();
            leds.signal_not_boosting();
        },
        SharedStateMachine::NestedState::Testing);
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
    if (spi.has_received_force_dc_link) {
        motor_driver_sensors.force_constant_dc_link_voltage(
            spi.forced_dc_link_voltage);

        spi.has_received_force_dc_link = false;
        spi.has_received_unlock_dc_link = false;
    } else if (spi.has_received_unlock_dc_link) {
        motor_driver_sensors.unlock_dc_link_voltage();

        spi.has_received_force_dc_link = false;
        spi.has_received_unlock_dc_link = false;
    }

    if (spi.has_received_commutation_settings) {
        motor_driver.set_frequency(spi.commutation_frequency_hz);
        motor_driver.set_dead_time(
            std::chrono::nanoseconds(spi.commutation_dead_time_ns));

        spi.has_received_commutation_settings = false;
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

void Board::update_idle() {}

void Board::update_ready() {}

void Board::update_boosting() {
    if (spi.has_received_start_current_control_order) {
        executor.set_d_current_reference(spi.requested_d_current_reference);
        executor.set_q_current_reference(spi.requested_q_current_reference);

        spi.has_received_start_current_control_order = false;
        spi.has_received_start_velocity_control_order = false;
    } else if (spi.has_received_start_velocity_control_order) {
        executor.set_velocity_reference(spi.requested_velocity_reference);

        spi.has_received_start_current_control_order = false;
        spi.has_received_start_velocity_control_order = false;
    }
}

void Board::update_testing() {
    if (spi.has_received_start_test_pwm_order) {
        executor.set_duty_cycle_u(spi.requested_duty_cycle_u);
        executor.set_duty_cycle_v(spi.requested_duty_cycle_v);
        executor.set_duty_cycle_w(spi.requested_duty_cycle_w);

        spi.has_received_start_test_pwm_order = false;
    } else if (spi.has_received_start_emulated_movement_order) {
        executor.set_angluar_velocity(spi.requested_angular_velocity);

        spi.has_received_start_emulated_movement_order = false;
    }
}

}  // namespace BCU