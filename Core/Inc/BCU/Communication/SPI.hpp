#pragma once

#include "ST-LIB.hpp"
#include "Shared/SPI.hpp"
#include "Shared/StateMachine.hpp"

namespace BCU::Communication {

class SPI {
   public:
    StateMachine::state_id master_general_state{
        Shared::State::SharedStateMachine::GeneralState::Connecting};
    StateMachine::state_id master_nested_state{
        Shared::State::SharedStateMachine::NestedState::Idle};

    float requested_duty_cycle_u{0.0f};
    float requested_duty_cycle_v{0.0f};
    float requested_duty_cycle_w{0.0f};

    uint32_t requested_commutation_frequency_hz{0};
    uint32_t requested_dead_time_ns{0};

    float requested_modulation_index{0.0f};
    float requested_modulation_frequency_hz{0.0f};

    static bool has_received_start_test_pwm;
    static bool has_received_configure_commutation_parameters;
    static bool has_received_stop_control;
    static bool has_received_start_space_vector;

   private:
    static void on_start_test_pwm() { has_received_start_test_pwm = true; }

    static void on_configure_commutation_parameters() {
        has_received_configure_commutation_parameters = true;
    }

    static void on_stop_control() { has_received_stop_control = true; }

    static void on_start_space_vector() {
        has_received_start_space_vector = true;
    }

    uint8_t spi_id;

    SPIStackOrder *state_order;

    SPIStackOrder *start_test_pwm_order;

    SPIStackOrder *configure_commutation_parameters_order;

    SPIStackOrder *stop_control_order;

    SPIStackOrder *control_parameters_order;

    SPIStackOrder *start_space_vector_order;

   public:
    SPI(StateMachine::state_id *slave_general_state,
        StateMachine::state_id *slave_nested_state, float *duty_cycle_u,
        float *duty_cycle_v, float *duty_cycle_w);

    SPI(Pin &spi_ready_slave_pin, StateMachine::state_id *slave_general_state,
        StateMachine::state_id *slave_nested_state, float *duty_cycle_u,
        float *duty_cycle_v, float *duty_cycle_w);

    void start();

    void update();
};

};  // namespace BCU::Communication