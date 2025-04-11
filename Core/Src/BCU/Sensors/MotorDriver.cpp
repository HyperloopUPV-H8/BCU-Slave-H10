#include "BCU/Sensors/MotorDriver.hpp"

namespace BCU::Sensors {

MotorDriver::MotorDriver(
    Pin& dc_link_voltage_pin_1, Pin& dc_link_voltage_pin_2,
    Pin& dc_link_voltage_pin_3, Pin& dc_link_voltage_pin_4,
    Pin& motor_phase_current_u_1, Pin& motor_phase_current_v_1,
    Pin& motor_phase_current_w_1, Pin& motor_phase_current_u_2,
    Pin& motor_phase_current_v_2, Pin& motor_phase_current_w_2,
    Pin& motor_phase_current_u_3, Pin& motor_phase_current_v_3,
    Pin& motor_phase_current_w_3, Pin& motor_phase_current_u_4,
    Pin& motor_phase_current_v_4, Pin& motor_phase_current_w_4,
    Pin& gate_driver_fault_pin_1, Pin& gate_driver_fault_pin_2,
    Pin& gate_driver_fault_pin_3, Pin& gate_driver_fault_pin_4,
    Pin& gate_driver_ready_pin_1, Pin& gate_driver_ready_pin_2,
    Pin& gate_driver_ready_pin_3, Pin& gate_driver_ready_pin_4)
    : dc_link_voltage{{
          LinearSensor<float>(dc_link_voltage_pin_1, DC_LINK_VOLTAGE_SLOPE,
                              DC_LINK_VOLTAGE_OFFSET,
                              &dc_link_voltage_value[0]),
          LinearSensor<float>(dc_link_voltage_pin_2, DC_LINK_VOLTAGE_SLOPE,
                              DC_LINK_VOLTAGE_OFFSET,
                              &dc_link_voltage_value[1]),
          LinearSensor<float>(dc_link_voltage_pin_3, DC_LINK_VOLTAGE_SLOPE,
                              DC_LINK_VOLTAGE_OFFSET,
                              &dc_link_voltage_value[2]),
          LinearSensor<float>(dc_link_voltage_pin_4, DC_LINK_VOLTAGE_SLOPE,
                              DC_LINK_VOLTAGE_OFFSET,
                              &dc_link_voltage_value[3]),
      }},
      motor_phase_current{{
          {LinearSensor<float>(
               motor_phase_current_u_1, MOTOR_PHASE_CURRENT_SLOPE,
               MOTOR_PHASE_CURRENT_OFFSET, &motor_phase_current_value[0][0]),
           LinearSensor<float>(
               motor_phase_current_v_1, MOTOR_PHASE_CURRENT_SLOPE,
               MOTOR_PHASE_CURRENT_OFFSET, &motor_phase_current_value[0][1]),
           LinearSensor<float>(
               motor_phase_current_w_1, MOTOR_PHASE_CURRENT_SLOPE,
               MOTOR_PHASE_CURRENT_OFFSET, &motor_phase_current_value[0][2])},
          {LinearSensor<float>(
               motor_phase_current_u_2, MOTOR_PHASE_CURRENT_SLOPE,
               MOTOR_PHASE_CURRENT_OFFSET, &motor_phase_current_value[1][0]),
           LinearSensor<float>(
               motor_phase_current_v_2, MOTOR_PHASE_CURRENT_SLOPE,
               MOTOR_PHASE_CURRENT_OFFSET, &motor_phase_current_value[1][1]),
           LinearSensor<float>(
               motor_phase_current_w_2, MOTOR_PHASE_CURRENT_SLOPE,
               MOTOR_PHASE_CURRENT_OFFSET, &motor_phase_current_value[1][2])},
          {LinearSensor<float>(
               motor_phase_current_u_3, MOTOR_PHASE_CURRENT_SLOPE,
               MOTOR_PHASE_CURRENT_OFFSET, &motor_phase_current_value[2][0]),
           LinearSensor<float>(
               motor_phase_current_v_3, MOTOR_PHASE_CURRENT_SLOPE,
               MOTOR_PHASE_CURRENT_OFFSET, &motor_phase_current_value[2][1]),
           LinearSensor<float>(
               motor_phase_current_w_3, MOTOR_PHASE_CURRENT_SLOPE,
               MOTOR_PHASE_CURRENT_OFFSET, &motor_phase_current_value[2][2])},
          {LinearSensor<float>(
               motor_phase_current_u_4, MOTOR_PHASE_CURRENT_SLOPE,
               MOTOR_PHASE_CURRENT_OFFSET, &motor_phase_current_value[3][0]),
           LinearSensor<float>(
               motor_phase_current_v_4, MOTOR_PHASE_CURRENT_SLOPE,
               MOTOR_PHASE_CURRENT_OFFSET, &motor_phase_current_value[3][1]),
           LinearSensor<float>(
               motor_phase_current_w_4, MOTOR_PHASE_CURRENT_SLOPE,
               MOTOR_PHASE_CURRENT_OFFSET, &motor_phase_current_value[3][2])},
      }},
      gate_driver_fault{{
          DigitalSensor(gate_driver_fault_pin_1, &gate_driver_fault_state[0]),
          DigitalSensor(gate_driver_fault_pin_2, &gate_driver_fault_state[1]),
          DigitalSensor(gate_driver_fault_pin_3, &gate_driver_fault_state[2]),
          DigitalSensor(gate_driver_fault_pin_4, &gate_driver_fault_state[3]),
      }},
      gate_driver_ready{{
          DigitalSensor(gate_driver_ready_pin_1, &gate_driver_ready_state[0]),
          DigitalSensor(gate_driver_ready_pin_2, &gate_driver_ready_state[1]),
          DigitalSensor(gate_driver_ready_pin_3, &gate_driver_ready_state[2]),
          DigitalSensor(gate_driver_ready_pin_4, &gate_driver_ready_state[3]),
      }} {}

void MotorDriver::read_dc_link_voltage() {
    float average_value{0};
    for (auto& sensor : dc_link_voltage) {
        sensor.read();
        average_value += *sensor.get_value_pointer();
    }
    average_value /= dc_link_voltage.size();

    if (!force_dc_link_voltage)
        computed_dc_link_voltage =
            dc_link_voltage_average.compute(average_value);
}

void MotorDriver::force_constant_dc_link_voltage(float voltage) {
    force_dc_link_voltage = true;
    computed_dc_link_voltage = voltage;
}

float MotorDriver::get_dc_link_voltage() const {
    return computed_dc_link_voltage;
}

void MotorDriver::unlock_dc_link_voltage() { force_dc_link_voltage = false; }

void MotorDriver::read_motor_phase_current() {
    for (auto& motor : motor_phase_current) {
        for (auto& sensor : motor) {
            sensor.read();
        }
    }
}

void MotorDriver::read_gate_driver_fault() {
    for (auto& input : gate_driver_fault) {
        input.read();
    }
}

void MotorDriver::read_gate_driver_ready() {
    for (auto& input : gate_driver_ready) {
        input.read();
    }
}

const float& MotorDriver::get_dc_link_voltage(uint8_t motor_id) const {
    return dc_link_voltage_value[motor_id];
}

float MotorDriver::get_motor_phase_u_current() const {
    float average{0};
    for (auto& phases : motor_phase_current_value) {
        average += phases[0];
    }
    return average / motor_phase_current_value.size();
}

const float& MotorDriver::get_motor_phase_u_current(uint8_t motor_id) const {
    return motor_phase_current_value[motor_id][0];
}

float MotorDriver::get_motor_phase_v_current() const {
    float average{0};
    for (auto& phases : motor_phase_current_value) {
        average += phases[1];
    }
    return average / motor_phase_current_value.size();
}

const float& MotorDriver::get_motor_phase_v_current(uint8_t motor_id) const {
    return motor_phase_current_value[motor_id][1];
}

float MotorDriver::get_motor_phase_w_current() const {
    float average{0};
    for (auto& phases : motor_phase_current_value) {
        average += phases[2];
    }
    return average / motor_phase_current_value.size();
}

const float& MotorDriver::get_motor_phase_w_current(uint8_t motor_id) const {
    return motor_phase_current_value[motor_id][2];
}

const PinState& MotorDriver::get_gate_driver_fault(uint8_t motor_id) const {
    return gate_driver_fault_state[motor_id];
}

const PinState& MotorDriver::get_gate_driver_ready(uint8_t motor_id) const {
    return gate_driver_ready_state[motor_id];
}

float* MotorDriver::get_dc_link_voltage_ptr() {
    return &computed_dc_link_voltage;
}

float* MotorDriver::get_dc_link_voltage_ptr(uint8_t motor_id) {
    return &dc_link_voltage_value[motor_id];
}

float* MotorDriver::get_motor_phase_u_current_ptr(uint8_t motor_id) {
    return &motor_phase_current_value[motor_id][0];
}

float* MotorDriver::get_motor_phase_v_current_ptr(uint8_t motor_id) {
    return &motor_phase_current_value[motor_id][1];
}

float* MotorDriver::get_motor_phase_w_current_ptr(uint8_t motor_id) {
    return &motor_phase_current_value[motor_id][2];
}

PinState* MotorDriver::get_gate_driver_fault_ptr(uint8_t motor_id) {
    return &gate_driver_fault_state[motor_id];
}

PinState* MotorDriver::get_gate_driver_ready_ptr(uint8_t motor_id) {
    return &gate_driver_ready_state[motor_id];
}

};  // namespace BCU::Sensors
