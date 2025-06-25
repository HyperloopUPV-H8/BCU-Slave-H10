#include "BCU/Pinout.hpp"

#define UART_TX_M_RX_S PD6
#define UART_RX_M_TX_S PD5

#define S_TXD_u PD1
#define S_RXD_u PD0
#define SLNT_Slave PA8

#define SPI_CS PD3
#define SPI_MOSI PC12
#define SPI_MISO PC11
#define SPI_CLK PC10

#define GD_Reset_1_u PG12
#define GD_Reset_2_u PG11
#define GD_Reset_3_u PG10
#define GD_Reset_4_u PG9

#define ENABLE_BUFFER_S_1 PD12
#define ENABLE_BUFFER_S_2 PD13
#define ENABLE_BUFFER_S_3 PD14

#define READY_1 PG1
#define READY_2 PG0
#define READY_3 PF15
#define READY_4 PE7

#define FAULT_1 PF2
#define FAULT_2 PF3
#define FAULT_3 PF4
#define FAULT_4 PF5

#define LED_OPERATIONAL_S PG8
#define LED_FAULT_S PG7
#define LED_CAN_S PG6
#define LED_FLASH_S PG5
#define LED_SLEEP_S PG4

#define EXTI_M_GPIO5 PG2
#define EXTI_S_GPIO5 PD2
#define GPIO4 PD11
#define GPIO3 PD10
#define GPIO2 PD9
#define GPIO1 PD8

#define Phase_U_PWM_A_u PE13
#define Phase_U_PWM_N_A_u PE12
#define Phase_V_PWM_A_u PE11
#define Phase_V_PWM_N_A_u PE10
#define Phase_W_PWM_A_u PC6
#define Phase_W_PWM_N_A_u PA7

#define Phase_U_PWM_B_u PE9
#define Phase_U_PWM_N_B_u PE8
#define Phase_V_PWM_B_u PC8
#define Phase_V_PWM_N_B_u PB15
#define Phase_W_PWM_B_u PC7
#define Phase_W_PWM_N_B_u PB14

#define SENSE1 PA2
#define SENSE2 PA3
#define SENSE3 PC1
#define SENSE4 PC0
#define SENSE5 PA6
#define SENSE6 PC4
#define SENSE7 PA4
#define SENSE8 PA5
#define SENSE9 PF13
#define SENSE10 PF14
#define SENSE11 PB0
#define SENSE12 PB1

#define A_Speetec1_u PF0
#define B_Speetec1_u PF1
#define A_Speetec2_u PF12
#define B_Speetec2_u PF11
#define A_Speetec3_u PA1
#define B_Speetec3_u PA0

#define VBAT1 PF9
#define VBAT2 PF6
#define VBAT3 PF8
#define VBAT4 PF7

namespace BCU::Pinout {
Pin& uart_tx_pin{UART_RX_M_TX_S};
Pin& uart_rx_pin{UART_TX_M_RX_S};

Pin& spi_chip_select_pin{SPI_CS};
Pin& spi_mosi_pin{SPI_MOSI};
Pin& spi_miso_pin{SPI_MISO};
Pin& spi_clock_pin{SPI_CLK};
Pin& spi_ready_slave_pin{GPIO1};

Pin& can_tx_pin{S_TXD_u};
Pin& can_rx_pin{S_RXD_u};
Pin& can_silent_mode_pin{SLNT_Slave};

Pin& led_operational_pin{LED_OPERATIONAL_S};
Pin& led_fault_pin{LED_FAULT_S};
Pin& led_can_pin{LED_CAN_S};
Pin& led_flash_pin{LED_FLASH_S};
Pin& led_sleep_pin{LED_SLEEP_S};

Pin& gate_driver_1_reset_pin{GD_Reset_1_u};
Pin& gate_driver_2_reset_pin{GD_Reset_2_u};
Pin& gate_driver_3_reset_pin{GD_Reset_3_u};
Pin& gate_driver_4_reset_pin{GD_Reset_4_u};

Pin& buffer_1_enable_pin{ENABLE_BUFFER_S_1};
Pin& buffer_2_enable_pin{ENABLE_BUFFER_S_2};
Pin& buffer_3_enable_pin{ENABLE_BUFFER_S_3};

Pin& gate_driver_1_ready_pin{READY_1};
Pin& gate_driver_2_ready_pin{READY_2};
Pin& gate_driver_3_ready_pin{READY_3};
Pin& gate_driver_4_ready_pin{READY_4};

Pin& gate_driver_1_fault_pin{FAULT_1};
Pin& gate_driver_2_fault_pin{FAULT_2};
Pin& gate_driver_3_fault_pin{FAULT_3};
Pin& gate_driver_4_fault_pin{FAULT_4};

Pin& phase_u_pwm_a_pin{Phase_U_PWM_A_u};
Pin& phase_u_negated_pwm_a_pin{Phase_U_PWM_N_A_u};
Pin& phase_v_pwm_a_pin{Phase_V_PWM_A_u};
Pin& phase_v_negated_pwm_a_pin{Phase_V_PWM_N_A_u};
Pin& phase_w_pwm_a_pin{Phase_W_PWM_A_u};
Pin& phase_w_negated_pwm_a_pin{Phase_W_PWM_N_A_u};

Pin& phase_u_pwm_b_pin{Phase_U_PWM_B_u};
Pin& phase_u_negated_pwm_b_pin{Phase_U_PWM_N_B_u};
Pin& phase_v_pwm_b_pin{Phase_V_PWM_B_u};
Pin& phase_v_negated_pwm_b_pin{Phase_V_PWM_N_B_u};
Pin& phase_w_pwm_b_pin{Phase_W_PWM_B_u};
Pin& phase_w_negated_pwm_b_pin{Phase_W_PWM_N_B_u};

Pin& phase_current_u_1_pin{SENSE1};
Pin& phase_current_v_1_pin{SENSE2};
Pin& phase_current_w_1_pin{SENSE3};

Pin& phase_current_u_2_pin{SENSE4};
Pin& phase_current_v_2_pin{SENSE5};
Pin& phase_current_w_2_pin{SENSE6};

Pin& phase_current_u_3_pin{SENSE7};
Pin& phase_current_v_3_pin{SENSE8};
Pin& phase_current_w_3_pin{SENSE9};

Pin& phase_current_u_4_pin{SENSE10};
Pin& phase_current_v_4_pin{SENSE11};
Pin& phase_current_w_4_pin{SENSE12};

Pin& speetec_1_1_pin{A_Speetec1_u};
Pin& speetec_1_2_pin{B_Speetec1_u};

Pin& speetec_2_1_pin{A_Speetec2_u};
Pin& speetec_2_2_pin{B_Speetec2_u};

Pin& speetec_3_1_pin{A_Speetec3_u};
Pin& speetec_3_2_pin{B_Speetec3_u};

Pin& dc_link_voltage_1_pin{VBAT1};
Pin& dc_link_voltage_2_pin{VBAT2};
Pin& dc_link_voltage_3_pin{VBAT3};
Pin& dc_link_voltage_4_pin{VBAT4};

};  // namespace BCU::Pinout