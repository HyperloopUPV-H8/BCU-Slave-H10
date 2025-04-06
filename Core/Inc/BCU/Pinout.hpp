#pragma once

#include "ST-LIB.hpp"

namespace BCU::Pinout {

extern Pin& uart_tx_pin;
extern Pin& uart_rx_pin;

extern Pin& spi_chip_select_pin;
extern Pin& spi_mosi_pin;
extern Pin& spi_miso_pin;
extern Pin& spi_clock_pin;
extern Pin& spi_ready_slave_pin;

extern Pin& can_tx_pin;
extern Pin& can_rx_pin;
extern Pin& can_silent_mode_pin;

extern Pin& led_operational_pin;
extern Pin& led_fault_pin;
extern Pin& led_can_pin;
extern Pin& led_flash_pin;
extern Pin& led_sleep_pin;

extern Pin& gate_driver_1_reset_pin;
extern Pin& gate_driver_2_reset_pin;
extern Pin& gate_driver_3_reset_pin;
extern Pin& gate_driver_4_reset_pin;

extern Pin& buffer_1_enable_pin;
extern Pin& buffer_2_enable_pin;
extern Pin& buffer_3_enable_pin;

extern Pin& gate_driver_1_ready_pin;
extern Pin& gate_driver_2_ready_pin;
extern Pin& gate_driver_3_ready_pin;
extern Pin& gate_driver_4_ready_pin;

extern Pin& gate_driver_1_fault_pin;
extern Pin& gate_driver_2_fault_pin;
extern Pin& gate_driver_3_fault_pin;
extern Pin& gate_driver_4_fault_pin;

extern Pin& phase_u_pwm_a_pin;
extern Pin& phase_u_negated_pwm_a_pin;
extern Pin& phase_v_pwm_a_pin;
extern Pin& phase_v_negated_pwm_a_pin;
extern Pin& phase_w_pwm_a_pin;
extern Pin& phase_w_negated_pwm_a_pin;

extern Pin& phase_u_pwm_b_pin;
extern Pin& phase_u_negated_pwm_b_pin;
extern Pin& phase_v_pwm_b_pin;
extern Pin& phase_v_negated_pwm_b_pin;
extern Pin& phase_w_pwm_b_pin;
extern Pin& phase_w_negated_pwm_b_pin;

extern Pin& phase_current_u_1_pin;
extern Pin& phase_current_v_1_pin;
extern Pin& phase_current_w_1_pin;

extern Pin& phase_current_u_2_pin;
extern Pin& phase_current_v_2_pin;
extern Pin& phase_current_w_2_pin;

extern Pin& phase_current_u_3_pin;
extern Pin& phase_current_v_3_pin;
extern Pin& phase_current_w_3_pin;

extern Pin& phase_current_u_4_pin;
extern Pin& phase_current_v_4_pin;
extern Pin& phase_current_w_4_pin;

extern Pin& speetec_1_1_pin;
extern Pin& speetec_1_2_pin;

extern Pin& speetec_2_1_pin;
extern Pin& speetec_2_2_pin;

extern Pin& speetec_3_1_pin;
extern Pin& speetec_3_2_pin;

extern Pin& dc_link_voltage_1_pin;
extern Pin& dc_link_voltage_2_pin;
extern Pin& dc_link_voltage_3_pin;
extern Pin& dc_link_voltage_4_pin;

};  // namespace BCU::Pinout