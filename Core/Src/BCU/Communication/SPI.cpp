#include "BCU/Communication/SPI.hpp"

namespace BCU::Communication {

SPI::SPI(StateMachine::state_id *slave_general_state,
         StateMachine::state_id *slave_nested_state, float *duty_cycle_u,
         float *duty_cycle_v, float *duty_cycle_w,
         float *average_dc_link_voltage, float *dc_link_voltage_1,
         float *dc_link_voltage_2, float *dc_link_voltage_3,
         float *dc_link_voltage_4)
    : spi_id(::SPI::inscribe(::SPI::spi3)),
      state_order(Shared::Communication::create_state_order(
          &master_general_state, &master_nested_state, slave_general_state,
          slave_nested_state)),
      start_test_pwm_order(Shared::Communication::create_start_test_pwm_order(
          &requested_duty_cycle_u, &requested_duty_cycle_v,
          &requested_duty_cycle_w)),
      configure_commutation_parameters_order(
          Shared::Communication::create_configure_commutation_parameters_order(
              &requested_commutation_frequency_hz, &requested_dead_time_ns)),
      stop_control_order(Shared::Communication::create_stop_control_order()),
      control_parameters_order(
          Shared::Communication::create_control_parameters_order(
              duty_cycle_u, duty_cycle_v, duty_cycle_w)),
      start_space_vector_order(
          Shared::Communication::create_start_space_vector_order(
              &requested_modulation_index, &requested_modulation_frequency_hz)),
      fix_dc_link_voltage_order(
          Shared::Communication::create_fix_dc_link_voltage_order(
              &requested_dc_link_voltage)),
      unfix_dc_link_voltage_order(
          Shared::Communication::create_unfix_dc_link_voltage_order()),
      dc_link_order(Shared::Communication::create_dc_link_order(
          average_dc_link_voltage, dc_link_voltage_1, dc_link_voltage_2,
          dc_link_voltage_3, dc_link_voltage_4)) {
    start_test_pwm_order->set_callback(on_start_test_pwm);
    configure_commutation_parameters_order->set_callback(
        on_configure_commutation_parameters);
    stop_control_order->set_callback(on_stop_control);
    start_space_vector_order->set_callback(on_start_space_vector);
    fix_dc_link_voltage_order->set_callback(on_fix_dc_link_voltage);
    unfix_dc_link_voltage_order->set_callback(on_unfix_dc_link_voltage);
}

SPI::SPI(Pin &spi_ready_slave_pin, StateMachine::state_id *slave_general_state,
         StateMachine::state_id *slave_nested_state, float *duty_cycle_u,
         float *duty_cycle_v, float *duty_cycle_w,
         float *average_dc_link_voltage, float *dc_link_voltage_1,
         float *dc_link_voltage_2, float *dc_link_voltage_3,
         float *dc_link_voltage_4)
    : SPI(slave_general_state, slave_nested_state, duty_cycle_u, duty_cycle_v,
          duty_cycle_w, average_dc_link_voltage, dc_link_voltage_1,
          dc_link_voltage_2, dc_link_voltage_3, dc_link_voltage_4) {
    ::SPI::assign_RS(spi_id, spi_ready_slave_pin);
}

void SPI::start() { ::SPI::slave_listen_Orders(spi_id); }

void SPI::update() { ::SPI::Order_update(); }
};  // namespace BCU::Communication