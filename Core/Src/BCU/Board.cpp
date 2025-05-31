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

    spi.start();

    Time::register_low_precision_alarm(
        1, [&]() { protection_manager.update_low_frequency(); });

    Time::register_mid_precision_alarm(
        1000, [&]() { motor_sensors.read_dc_link_voltage(); });
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

void Board::update_operational_ready() {}

void Board::update_operational_boosting() {}

void Board::update_operational_testing() {}

void Board::initialize_state_machine() {
    // General State Machine

    //     Transitions

    state_machine.general.add_transition(
        GeneralState::Connecting, GeneralState::Operational, [&]() {
            return spi.master_general_state == GeneralState::Operational;
        });

    // Operational State Machine

    //     Transitions

    //     Enter Actions

    state_machine.nested.add_enter_action(
        [&]() {
            leds.signal_inverter_off();
            motor_driver.turn_off();
            executor.stop();
        },
        OperationalState::Idle);

    state_machine.nested.add_enter_action(
        [&]() {
            motor_driver.set_duty_cycle_u(50.0f);
            motor_driver.set_duty_cycle_v(50.0f);
            motor_driver.set_duty_cycle_w(50.0f);
        },
        OperationalState::Precharge);

    state_machine.nested.add_enter_action([&]() {
        leds.signal_inverter_on();
        motor_driver.turn_on();
        executor.start_space_vector(0.8f, 1000.0f);
    }, );

    //     Exit Actions

    state_machine.nested.add_exit_action(
        [&]() {
            leds.signal_inverter_on();
            motor_driver.turn_on();
        },
        OperationalState::Idle);
}

}  // namespace BCU