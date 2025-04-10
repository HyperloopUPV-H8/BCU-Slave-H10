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

    static void on_start_velocity_control_order() {
        has_received_start_velocity_control_order = true;
    }
    static void on_start_current_control_order() {
        has_received_start_current_control_order = true;
    }
    static void on_start_emulated_movement_order() {
        has_received_start_emulated_movement_order = true;
    }
    static void on_start_test_pwm_order() {
        has_received_start_test_pwm_order = true;
    }

    static void on_stop_control_order() {
        enable_booster = false;
        has_received_stop_control_order = true;
    }

    static void on_enable_booster() { enable_booster = true; }

    static void on_force_dc_link() { has_received_force_dc_link = true; }
    static void on_unlock_dc_link() { has_received_unlock_dc_link = true; }

    static void on_commutation_settings() {
        has_received_commutation_settings = true;
    }

    static bool has_received_start_velocity_control_order;
    static bool has_received_start_current_control_order;
    static bool has_received_start_emulated_movement_order;
    static bool has_received_start_test_pwm_order;
    static bool has_received_stop_control_order;
    static bool enable_booster;
    static bool has_received_force_dc_link;
    static bool has_received_unlock_dc_link;
    static bool has_received_commutation_settings;

    double requested_velocity_reference{0.0};
    double requested_d_current_reference{0.0};
    double requested_q_current_reference{0.0};
    double requested_angular_velocity{0.0};
    double requested_duty_cycle_u{0.0};
    double requested_duty_cycle_v{0.0};
    double requested_duty_cycle_w{0.0};
    float forced_dc_link_voltage{0.0};
    uint32_t commutation_frequency_hz{0};
    uint32_t commutation_dead_time_ns{0};

   private:
    uint8_t spi_id;

    SPIStackOrder *state_order;

    SPIStackOrder *control_parameters_order;

    SPIStackOrder *start_velocity_control_order;
    SPIStackOrder *start_current_control_order;
    SPIStackOrder *start_emulated_movement_order;
    SPIStackOrder *start_test_pwm_order;

    SPIStackOrder *stop_control_order;

    SPIStackOrder *enable_booster_order;

    SPIStackOrder *encoder_order;

    SPIStackOrder *force_dc_link_order;
    SPIStackOrder *unlock_dc_link_order;

    SPIStackOrder *commutation_settings_order;

    SPIStackOrder *motor_driver_order;

   public:
    SPI(StateMachine::state_id *slave_general_state,
        StateMachine::state_id *slave_nested_state, double *velocity_reference,
        double *velocity_error, double *u_current_measurement,
        double *v_current_measurement, double *w_current_measurement,
        double *electrical_angle, double *d_current_reference,
        double *d_current_measurement, double *d_current_error,
        double *q_current_reference, double *q_current_measurement,
        double *q_current_error, double *three_phase_unbalance,
        double *d_target_voltage, double *q_target_voltage,
        double *u_target_voltage, double *v_target_voltage,
        double *w_target_voltage, double *u_output_voltage,
        double *v_output_voltage, double *w_output_voltage,
        double *u_duty_cycle, double *v_duty_cycle, double *w_duty_cycle,
        double *angular_velocity, std::array<double *, 3> position,
        std::array<double *, 3> velocity, std::array<double *, 3> acceleration,
        std::array<Shared::Communication::Direction *, 3> direction,
        double *average_position, double *max_velocity, bool *is_detecting,
        std::array<float *, 4> dc_link_voltage,
        std::array<std::array<float *, 3>, 4> motor_phase_current,
        std::array<PinState *, 4> gate_driver_fault,
        std::array<PinState *, 4> gate_driver_ready);

    SPI(Pin &spi_ready_slave_pin, StateMachine::state_id *slave_general_state,
        StateMachine::state_id *slave_nested_state, double *velocity_reference,
        double *velocity_error, double *u_current_measurement,
        double *v_current_measurement, double *w_current_measurement,
        double *electrical_angle, double *d_current_reference,
        double *d_current_measurement, double *d_current_error,
        double *q_current_reference, double *q_current_measurement,
        double *q_current_error, double *three_phase_unbalance,
        double *d_target_voltage, double *q_target_voltage,
        double *u_target_voltage, double *v_target_voltage,
        double *w_target_voltage, double *u_output_voltage,
        double *v_output_voltage, double *w_output_voltage,
        double *u_duty_cycle, double *v_duty_cycle, double *w_duty_cycle,
        double *angular_velocity, std::array<double *, 3> position,
        std::array<double *, 3> velocity, std::array<double *, 3> acceleration,
        std::array<Shared::Communication::Direction *, 3> direction,
        double *average_position, double *max_velocity, bool *is_detecting,
        std::array<float *, 4> dc_link_voltage,
        std::array<std::array<float *, 3>, 4> motor_phase_current,
        std::array<PinState *, 4> gate_driver_fault,
        std::array<PinState *, 4> gate_driver_ready);

    void start();

    void update();
};

};  // namespace BCU::Communication