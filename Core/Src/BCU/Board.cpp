#include "BCU/Board.hpp"

namespace BCU {

ProtectionManagerHandle::ProtectionManagerHandle(
    StateMachine& state_machine, StateMachine::state_id fault_state) {
    ProtectionManager::link_state_machine(state_machine, fault_state);
    ProtectionManager::add_standard_protections();
}

void ProtectionManagerHandle::start() { ProtectionManager::initialize(); }

void ProtectionManagerHandle::update_high_frequency() {
    ProtectionManager::check_high_frequency_protections();
}

void ProtectionManagerHandle::update_low_frequency() {
    ProtectionManager::check_protections();
}

STLIBHandle::STLIBHandle(string mac, string ip, string subnet_mask,
                         string gateway, UART::Peripheral& printf_peripheral) {
    STLIB::start(mac, ip, subnet_mask, gateway, printf_peripheral);
}

void STLIBHandle::update() { STLIB::update(); }

Board::Board() {
    motor_driver.turn_off();
    motor_driver.set_commutation_frequency_hz(30000);
    motor_driver.set_dead_time_ns(300);

    leds.signal_connecting();

    initialize_state_machine();
    initialize_protections();

    position_sense.turn_on();

    spi.start();

    Time::register_low_precision_alarm(1, [&]() {
        should_update_1khz = true;
        should_update_low_frequency = true;
    });

    Time::register_low_precision_alarm(16,
                                       [&]() { should_update_60hz = true; });

    Time::register_low_precision_alarm(100,
                                       [&]() { should_update_encoder = true; });
}

void Board::update() {
    switch (state_machine.general.current_state) {
        case GeneralState::Connecting:
            update_connecting();
            break;
        case GeneralState::Operational:
            update_operational();
            break;
        case GeneralState::Fault:
            update_fault();
            break;
    }

    if (should_update_low_frequency) {
        should_update_low_frequency = false;
        protection_manager.update_low_frequency();
    }

    if (should_update_1khz) {
        should_update_1khz = false;
        current_sense.read();
    }

    if (should_update_60hz) {
        should_update_60hz = false;
        motor_sensors.read_dc_link_voltage();
        motor_sensors.read_gate_driver_fault();
        motor_sensors.read_gate_driver_ready();
    }

    if (should_update_encoder) {
        position_sense.read();
    }

    spi.update();

    state_machine.general.check_transitions();
    protection_manager.update_high_frequency();
    stlib.update();
}

void Board::update_connecting() {}

void Board::update_operational() {
    switch (state_machine.nested.current_state) {
        case OperationalState::Idle:
            update_operational_idle();
            break;
        case OperationalState::Precharge:
            update_operational_precharge();
            break;
        case OperationalState::Ready:
            update_operational_ready();
            break;
        case OperationalState::Boosting:
            update_operational_boosting();
            break;
        case OperationalState::Testing:
            update_operational_testing();
            break;
    }
}

void Board::update_fault() {}

void Board::update_operational_idle() {}

void Board::update_operational_precharge() {}

void Board::update_operational_ready() {
    if (spi.has_received_enable_current_control) {
        executor.configure_current_control(spi.requested_current_d,
                                           spi.requested_current_q);

        spi.has_received_enable_current_control = false;
        spi.has_received_enable_velocity_control = false;
    } else if (spi.has_received_enable_velocity_control) {
        executor.configure_speed_control(spi.requested_velocity);

        spi.has_received_enable_velocity_control = false;
    }
}

void Board::update_operational_boosting() {}

void Board::update_operational_testing() {
    if (spi.has_received_test_pwm) {
        executor.start_test_pwm(spi.requested_duty_cycle_u,
                                spi.requested_duty_cycle_v,
                                spi.requested_duty_cycle_w);

        spi.has_received_test_pwm = false;
        spi.has_received_test_space_vector = false;
    } else if (spi.has_received_test_space_vector) {
        executor.start_space_vector(spi.requested_modulation_index,
                                    spi.requested_modulation_frequency_hz);

        spi.has_received_test_space_vector = false;
    }
}

void Board::initialize_state_machine() {
    // General State Machine

    //     Transitions

    state_machine.general.add_transition(
        GeneralState::Connecting, GeneralState::Operational, [&]() {
            return spi.master_general_state == GeneralState::Operational;
        });

    //     Enter Actions

    state_machine.general.add_enter_action([&]() { leds.signal_connecting(); },
                                           GeneralState::Connecting);

    state_machine.general.add_enter_action([&]() { leds.signal_operational(); },
                                           GeneralState::Operational);

    state_machine.general.add_enter_action([&]() { leds.signal_fault(); },
                                           GeneralState::Fault);

    // Operational State Machine

    //     Transitions

    state_machine.nested.add_transition(
        OperationalState::Idle, OperationalState::Precharge, [&]() {
            return spi.master_general_state == GeneralState::Operational &&
                   spi.master_nested_state == OperationalState::Precharge;
        });

    state_machine.nested.add_transition(
        OperationalState::Precharge, OperationalState::Ready, [&]() {
            return spi.has_received_enable_current_control ||
                   spi.has_received_enable_velocity_control;
        });

    state_machine.nested.add_transition(
        OperationalState::Precharge, OperationalState::Testing, [&]() {
            return spi.has_received_test_pwm ||
                   spi.has_received_test_space_vector;
        });

    state_machine.nested.add_transition(
        OperationalState::Precharge, OperationalState::Idle, [&]() {
            return spi.master_general_state == GeneralState::Operational &&
                   spi.master_nested_state == OperationalState::Idle;
        });

    state_machine.nested.add_transition(
        OperationalState::Testing, OperationalState::Precharge, [&]() {
            return spi.master_general_state == GeneralState::Operational &&
                   spi.master_nested_state == OperationalState::Precharge;
        });

    state_machine.nested.add_transition(
        OperationalState::Testing, OperationalState::Idle, [&]() {
            return spi.master_general_state == GeneralState::Operational &&
                   spi.master_nested_state == OperationalState::Idle;
        });

    state_machine.nested.add_transition(
        OperationalState::Ready, OperationalState::Boosting, [&]() {
            return (*position_sense.get_speed_ptr() >= MIN_SPEED_FOR_BOOSTER) &&
                   (spi.master_general_state == GeneralState::Operational &&
                    spi.master_nested_state == OperationalState::Boosting);
        });

    state_machine.nested.add_transition(
        OperationalState::Ready, OperationalState::Precharge, [&]() {
            return spi.master_general_state == GeneralState::Operational &&
                   spi.master_nested_state == OperationalState::Precharge;
        });

    state_machine.nested.add_transition(
        OperationalState::Ready, OperationalState::Idle, [&]() {
            return spi.master_general_state == GeneralState::Operational &&
                   spi.master_nested_state == OperationalState::Idle;
        });

    state_machine.nested.add_transition(
        OperationalState::Boosting, OperationalState::Precharge, [&]() {
            return (*position_sense.get_speed_ptr() < MIN_SPEED_FOR_BOOSTER) ||
                   (spi.master_general_state == GeneralState::Operational &&
                    spi.master_nested_state == OperationalState::Precharge);
        });

    state_machine.nested.add_transition(
        OperationalState::Boosting, OperationalState::Idle, [&]() {
            return spi.master_general_state == GeneralState::Operational &&
                   spi.master_nested_state == OperationalState::Idle;
        });

    //     Enter Actions

    state_machine.nested.add_enter_action(
        [&]() {
            motor_driver.turn_off();
            executor.stop();
            leds.signal_inverter_off();
        },
        OperationalState::Idle);

    state_machine.nested.add_enter_action(
        [&]() { executor.start_test_pwm(50.0f, 50.0f, 50.0f); },
        OperationalState::Precharge);

    state_machine.nested.add_enter_action(
        [&]() { executor.start_selected_control(); },
        OperationalState::Boosting);

    //     Exit Actions

    state_machine.nested.add_exit_action(
        [&]() {
            motor_driver.turn_on();
            leds.signal_inverter_on();
        },
        OperationalState::Idle);
}

void Board::initialize_protections() {
    add_high_frequency_protection(
        &spi.master_general_state,
        Boundary<StateMachine::state_id, EQUALS>(GeneralState::Fault));

    protection_manager.start();
}

}  // namespace BCU