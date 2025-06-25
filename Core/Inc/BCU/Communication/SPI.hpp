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

    float requested_modulation_index{0.0f};
    float requested_modulation_frequency_hz{0.0f};

    float requested_current_d{0.0f};
    float requested_current_q{0.0f};

    float requested_velocity{0.0f};

    inline static bool has_received_start_precharge{false};
    inline static bool has_received_test_pwm{false};
    inline static bool has_received_test_space_vector{false};
    inline static bool has_received_enable_current_control{false};
    inline static bool has_received_enable_velocity_control{false};
    inline static bool has_received_start_control{false};
    inline static bool has_received_stop_control{false};
    inline static bool has_received_disable{false};

   private:
    static void on_start_precharge() { has_received_start_precharge = true; }
    static void on_test_pwm() { has_received_test_pwm = true; }
    static void on_test_space_vector() {
        has_received_test_space_vector = true;
    }
    static void on_enable_current_control() {
        has_received_enable_current_control = true;
    }
    static void on_enable_velocity_control() {
        has_received_enable_velocity_control = true;
    }
    static void on_start_control() { has_received_start_control = true; }
    static void on_stop_control() { has_received_stop_control = true; }
    static void on_disable() { has_received_disable = true; }

    uint8_t spi_id;

    SPIStackOrder *state_order;
    SPIStackOrder *start_precharge_order;
    SPIStackOrder *test_pwm_order;
    SPIStackOrder *test_space_vector_order;
    SPIStackOrder *enable_current_control_order;
    SPIStackOrder *enable_velocity_control_order;
    SPIStackOrder *encoder_order;
    SPIStackOrder *start_control_order;
    SPIStackOrder *stop_control_order;
    SPIStackOrder *disable_order;

    SPIStackOrder *detailed_encoder_order;

   public:
    SPI(StateMachine::state_id *slave_general_state,
        StateMachine::state_id *slave_nested_state, double *position,
        double *velocity, double *acceleration, uint8_t* direction, double *position_1,
        double *velocity_1, double *acceleration_1, uint8_t *direction_1,
        double *position_2, double *velocity_2, double *acceleration_2,
        uint8_t *direction_2);

    SPI(Pin &spi_ready_slave_pin, StateMachine::state_id *slave_general_state,
        StateMachine::state_id *slave_nested_state, double *position,
        double *velocity, double *acceleration, uint8_t* direction, double *position_1,
        double *velocity_1, double *acceleration_1, uint8_t *direction_1,
        double *position_2, double *velocity_2, double *acceleration_2,
        uint8_t *direction_2);

    void start();

    void update();
};

};  // namespace BCU::Communication