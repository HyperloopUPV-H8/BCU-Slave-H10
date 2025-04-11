#include "BCU/Communication/SPI.hpp"

namespace BCU::Communication {

bool SPI::has_received_start_velocity_control_order{false};
bool SPI::has_received_start_current_control_order{false};
bool SPI::has_received_start_emulated_movement_order{false};
bool SPI::has_received_start_test_pwm_order{false};
bool SPI::has_received_stop_control_order{false};
bool SPI::enable_booster{false};
bool SPI::has_received_force_dc_link{false};
bool SPI::has_received_unlock_dc_link{false};
bool SPI::has_received_commutation_settings{false};

SPI::SPI(StateMachine::state_id *slave_general_state,
         StateMachine::state_id *slave_nested_state, float *velocity_reference,
         float *velocity_error, float *u_current_measurement,
         float *v_current_measurement, float *w_current_measurement,
         float *electrical_angle, float *d_current_reference,
         float *d_current_measurement, float *d_current_error,
         float *q_current_reference, float *q_current_measurement,
         float *q_current_error, float *three_phase_unbalance,
         float *d_target_voltage, float *q_target_voltage,
         float *u_target_voltage, float *v_target_voltage,
         float *w_target_voltage, float *u_output_voltage,
         float *v_output_voltage, float *w_output_voltage, float *u_duty_cycle,
         float *v_duty_cycle, float *w_duty_cycle, float *angular_velocity,
         std::array<float *, 3> position, std::array<float *, 3> velocity,
         std::array<float *, 3> acceleration,
         std::array<Shared::Communication::Direction *, 3> direction,
         float *average_position, float *max_velocity, bool *is_detecting,
         std::array<float *, 4> dc_link_voltage,
         std::array<std::array<float *, 3>, 4> motor_phase_current,
         std::array<PinState *, 4> gate_driver_fault,
         std::array<PinState *, 4> gate_driver_ready)
    : spi_id(::SPI::inscribe(::SPI::spi3)),
      state_order(Shared::Communication::create_state_order(
          &master_general_state, &master_nested_state, slave_general_state,
          slave_nested_state)),
      control_parameters_order(
          Shared::Communication::create_control_parameters_order(
              velocity_reference, velocity_error, u_current_measurement,
              v_current_measurement, w_current_measurement, electrical_angle,
              d_current_reference, d_current_measurement, d_current_error,
              q_current_reference, q_current_measurement, q_current_error,
              three_phase_unbalance, d_target_voltage, q_target_voltage,
              u_target_voltage, v_target_voltage, w_target_voltage,
              u_output_voltage, v_output_voltage, w_output_voltage,
              u_duty_cycle, v_duty_cycle, w_duty_cycle, angular_velocity)),
      start_velocity_control_order(
          Shared::Communication::create_start_velocity_control_order(
              &requested_velocity_reference)),
      start_current_control_order(
          Shared::Communication::create_start_current_control_order(
              &requested_d_current_reference, &requested_q_current_reference)),
      start_emulated_movement_order(
          Shared::Communication::create_start_emulated_movement_order(
              &requested_d_current_reference, &requested_q_current_reference,
              &requested_angular_velocity)),
      start_test_pwm_order(Shared::Communication::create_start_test_pwm_order(
          &requested_duty_cycle_u, &requested_duty_cycle_v,
          &requested_duty_cycle_w)),
      stop_control_order(Shared::Communication::create_stop_control_order()),
      enable_booster_order(
          Shared::Communication::create_enable_booster_order()),
      encoder_order(Shared::Communication::create_position_encoder_order(
          position, velocity, acceleration, direction, average_position,
          max_velocity, is_detecting)),
      force_dc_link_order(Shared::Communication::create_force_dc_link_order(
          &forced_dc_link_voltage)),
      unlock_dc_link_order(
          Shared::Communication::create_unlock_dc_link_order()),
      commutation_settings_order(
          Shared::Communication::create_commutation_settings_order(
              &commutation_frequency_hz, &commutation_dead_time_ns)),
      motor_driver_order(Shared::Communication::create_motor_driver_order(
          dc_link_voltage, motor_phase_current, gate_driver_fault,
          gate_driver_ready)) {
    start_velocity_control_order->set_callback(on_start_velocity_control_order);
    start_current_control_order->set_callback(on_start_current_control_order);
    start_emulated_movement_order->set_callback(
        on_start_emulated_movement_order);
    start_test_pwm_order->set_callback(on_start_test_pwm_order);
    stop_control_order->set_callback(on_stop_control_order);
    enable_booster_order->set_callback(on_enable_booster);
    force_dc_link_order->set_callback(on_force_dc_link);
    unlock_dc_link_order->set_callback(on_unlock_dc_link);
    commutation_settings_order->set_callback(on_commutation_settings);
}

SPI::SPI(Pin &spi_ready_slave_pin, StateMachine::state_id *slave_general_state,
         StateMachine::state_id *slave_nested_state, float *velocity_reference,
         float *velocity_error, float *u_current_measurement,
         float *v_current_measurement, float *w_current_measurement,
         float *electrical_angle, float *d_current_reference,
         float *d_current_measurement, float *d_current_error,
         float *q_current_reference, float *q_current_measurement,
         float *q_current_error, float *three_phase_unbalance,
         float *d_target_voltage, float *q_target_voltage,
         float *u_target_voltage, float *v_target_voltage,
         float *w_target_voltage, float *u_output_voltage,
         float *v_output_voltage, float *w_output_voltage, float *u_duty_cycle,
         float *v_duty_cycle, float *w_duty_cycle, float *angular_velocity,
         std::array<float *, 3> position, std::array<float *, 3> velocity,
         std::array<float *, 3> acceleration,
         std::array<Shared::Communication::Direction *, 3> direction,
         float *average_position, float *max_velocity, bool *is_detecting,
         std::array<float *, 4> dc_link_voltage,
         std::array<std::array<float *, 3>, 4> motor_phase_current,
         std::array<PinState *, 4> gate_driver_fault,
         std::array<PinState *, 4> gate_driver_ready)
    : SPI(slave_general_state, slave_nested_state, velocity_reference,
          velocity_error, u_current_measurement, v_current_measurement,
          w_current_measurement, electrical_angle, d_current_reference,
          d_current_measurement, d_current_error, q_current_reference,
          q_current_measurement, q_current_error, three_phase_unbalance,
          d_target_voltage, q_target_voltage, u_target_voltage,
          v_target_voltage, w_target_voltage, u_output_voltage,
          v_output_voltage, w_output_voltage, u_duty_cycle, v_duty_cycle,
          w_duty_cycle, angular_velocity, position, velocity, acceleration,
          direction, average_position, max_velocity, is_detecting,
          dc_link_voltage, motor_phase_current, gate_driver_fault,
          gate_driver_ready) {
    ::SPI::assign_RS(spi_id, spi_ready_slave_pin);
}

void SPI::start() { ::SPI::slave_listen_Orders(spi_id); }

void SPI::update() { ::SPI::Order_update(); }

};  // namespace BCU::Communication