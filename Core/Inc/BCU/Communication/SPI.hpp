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

    float requested_dc_link_voltage{0.0f};

    inline static bool has_received_start_test_pwm{false};
    inline static bool has_received_configure_commutation_parameters{false};
    inline static bool has_received_stop_control{false};
    inline static bool has_received_start_space_vector{false};
    inline static bool has_received_fix_dc_link_voltage{false};
    inline static bool has_received_unfix_dc_link_voltage{false};

   private:
    static void on_start_test_pwm() { has_received_start_test_pwm = true; }

    static void on_configure_commutation_parameters() {
        has_received_configure_commutation_parameters = true;
    }

    static void on_stop_control() { has_received_stop_control = true; }

    static void on_start_space_vector() {
        has_received_start_space_vector = true;
    }

    static void on_fix_dc_link_voltage() {
        has_received_fix_dc_link_voltage = true;
    }

    static void on_unfix_dc_link_voltage() {
        has_received_unfix_dc_link_voltage = true;
    }

    uint8_t spi_id;

    SPIStackOrder *state_order;

    SPIStackOrder *start_test_pwm_order;

    SPIStackOrder *configure_commutation_parameters_order;

    SPIStackOrder *stop_control_order;

    SPIStackOrder *control_parameters_order;

    SPIStackOrder *start_space_vector_order;

    SPIStackOrder *fix_dc_link_voltage_order;
    SPIStackOrder *unfix_dc_link_voltage_order;
    SPIStackOrder *dc_link_order;

   public:
    SPI(StateMachine::state_id *slave_general_state,
        StateMachine::state_id *slave_nested_state, float *duty_cycle_u,
        float *duty_cycle_v, float *duty_cycle_w,
        float *average_dc_link_voltage, float *dc_link_voltage_1,
        float *dc_link_voltage_2, float *dc_link_voltage_3,
        float *dc_link_voltage_4);

    SPI(Pin &spi_ready_slave_pin, StateMachine::state_id *slave_general_state,
        StateMachine::state_id *slave_nested_state, float *duty_cycle_u,
        float *duty_cycle_v, float *duty_cycle_w,
        float *average_dc_link_voltage, float *dc_link_voltage_1,
        float *dc_link_voltage_2, float *dc_link_voltage_3,
        float *dc_link_voltage_4);

    void start();

    void update();
};

};  // namespace BCU::Communication