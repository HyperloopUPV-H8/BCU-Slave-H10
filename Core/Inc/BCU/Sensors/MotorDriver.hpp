#pragma once

#include "ST-LIB.hpp"

namespace BCU::Sensors {

class MotorDriver {
    constexpr static float DC_LINK_VOLTAGE_SLOPE{1.0f};
    constexpr static float DC_LINK_VOLTAGE_OFFSET{0.0f};

    constexpr static float MOTOR_PHASE_CURRENT_SLOPE{1.0f};
    constexpr static float MOTOR_PHASE_CURRENT_OFFSET{0.0f};

    std::array<LinearSensor<float>, 4> dc_link_voltage;
    std::array<std::array<LinearSensor<float>, 3>, 4> motor_phase_current;

    std::array<DigitalSensor, 4> gate_driver_fault;
    std::array<DigitalSensor, 4> gate_driver_ready;

    std::array<float, 4> dc_link_voltage_value;
    std::array<std::array<float, 3>, 4> motor_phase_current_value;
    std::array<PinState, 4> gate_driver_fault_state;
    std::array<PinState, 4> gate_driver_ready_state;

    float computed_dc_link_voltage{0.0f};
    MovingAverage<50> dc_link_voltage_average;
    bool force_dc_link_voltage{false};

   public:
    MotorDriver(std::array<Pin &, 4> dc_link_voltage_pin,
                std::array<std::array<Pin &, 3>, 4> motor_phase_current_pin,
                std::array<Pin &, 4> gate_driver_fault_pin,
                std::array<Pin &, 4> gate_driver_ready_pin);

    void force_constant_dc_link_voltage(float voltage);
    void unlock_dc_link_voltage();

    void read_dc_link_voltage();
    void read_motor_phase_current();
    void read_gate_driver_fault();
    void read_gate_driver_ready();

    float get_dc_link_voltage() const;
    const float &get_dc_link_voltage(uint8_t motor_id) const;

    float get_motor_phase_u_current() const;
    const float &get_motor_phase_u_current(uint8_t motor_id) const;

    float get_motor_phase_v_current() const;
    const float &get_motor_phase_v_current(uint8_t motor_id) const;

    float get_motor_phase_w_current() const;
    const float &get_motor_phase_w_current(uint8_t motor_id) const;

    const PinState &get_gate_driver_fault(uint8_t motor_id) const;
    const PinState &get_gate_driver_ready(uint8_t motor_id) const;

    float *get_dc_link_voltage_ptr();
    float *get_dc_link_voltage_ptr(uint8_t motor_id);

    float *get_motor_phase_u_current_ptr(uint8_t motor_id);
    float *get_motor_phase_v_current_ptr(uint8_t motor_id);
    float *get_motor_phase_w_current_ptr(uint8_t motor_id);

    PinState *get_gate_driver_fault_ptr(uint8_t motor_id);
    PinState *get_gate_driver_ready_ptr(uint8_t motor_id);
};

};  // namespace BCU::Sensors