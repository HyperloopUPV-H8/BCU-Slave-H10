#include "BCU/Communication/SPI.hpp"

namespace BCU::Communication {

SPI::SPI(StateMachine::state_id *slave_general_state,
         StateMachine::state_id *slave_nested_state, double *position,
         double *velocity, double *acceleration, uint8_t *direction,
         double *position_1, double *velocity_1, double *acceleration_1,
         uint8_t *direction_1, double *position_2, double *velocity_2,
         double *acceleration_2, uint8_t *direction_2)
    : spi_id(::SPI::inscribe(::SPI::spi3)),
      state_order(Shared::Communication::create_state_order(
          &master_general_state, &master_nested_state, slave_general_state,
          slave_nested_state)),
      start_precharge_order(
          Shared::Communication::create_start_precharge_order()),
      test_pwm_order(Shared::Communication::create_test_pwm_order(
          &requested_duty_cycle_u, &requested_duty_cycle_v,
          &requested_duty_cycle_w)),
      test_space_vector_order(
          Shared::Communication::create_test_space_vector_order(
              &requested_modulation_index, &requested_modulation_frequency_hz)),
      enable_current_control_order(
          Shared::Communication::create_enable_current_control_order(
              &requested_current_d, &requested_current_q)),
      enable_velocity_control_order(
          Shared::Communication::create_enable_velocity_control_order(
              &requested_velocity)),
      encoder_order(Shared::Communication::create_encoder_order(
          position, velocity, acceleration, direction)),
      start_control_order(Shared::Communication::create_start_control_order()),
      stop_control_order(Shared::Communication::create_stop_control_order()),
      disable_order(Shared::Communication::create_disable_order()),
      detailed_encoder_order(
          Shared::Communication::create_detailed_encoder_order(
              position_1, velocity_1, acceleration_1, direction_1, position_2,
              velocity_2, acceleration_2, direction_2)) {
    start_precharge_order->set_callback(on_start_precharge);
    test_pwm_order->set_callback(on_test_pwm);
    test_space_vector_order->set_callback(on_test_space_vector);
    enable_current_control_order->set_callback(on_enable_current_control);
    enable_velocity_control_order->set_callback(on_enable_velocity_control);
    start_control_order->set_callback(on_start_control);
    stop_control_order->set_callback(on_stop_control);
    disable_order->set_callback(on_disable);
}

SPI::SPI(Pin &spi_ready_slave_pin, StateMachine::state_id *slave_general_state,
         StateMachine::state_id *slave_nested_state, double *position,
         double *velocity, double *acceleration, uint8_t *direction,
         double *position_1, double *velocity_1, double *acceleration_1,
         uint8_t *direction_1, double *position_2, double *velocity_2,
         double *acceleration_2, uint8_t *direction_2)
    : SPI(slave_general_state, slave_nested_state, position, velocity,
          acceleration, direction, position_1, velocity_1, acceleration_1,
          direction_1, position_2, velocity_2, acceleration_2, direction_2) {
    ::SPI::assign_RS(spi_id, spi_ready_slave_pin);
}

void SPI::start() { ::SPI::slave_listen_Orders(spi_id); }

void SPI::update() { ::SPI::Order_update(); }
};  // namespace BCU::Communication