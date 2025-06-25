#include "BCU/Control/Executor.hpp"

namespace BCU::Control {

Executor::Executor(Sensors::MotorSense &voltage_sense,
                   Sensors::CurrentSense &current_sense,
                   Sensors::PositionSense &speetec,
                   Actuators::MotorDriver &motor_driver)
    : voltage_sense(voltage_sense),
      current_sense(current_sense),
      speetec(speetec),
      motor_driver(motor_driver) {}

void Executor::stop() {
    motor_driver.turn_off();

    switch (mode) {
        case Mode::IDLE:
            break;
        case Mode::TEST_PWM:
            break;
        case Mode::SPACE_VECTOR:
            Time::unregister_mid_precision_alarm(control_alarm_id);
            break;
        case Mode::EMULATED_CURRENT_CONTROL:
            Time::unregister_mid_precision_alarm(control_alarm_id);
            break;
        case Mode::CURRENT_CONTROL:
            Time::unregister_mid_precision_alarm(control_alarm_id);
        case Mode::SPEED_CONTROL:
            Time::unregister_mid_precision_alarm(control_alarm_id);
            break;
    }

    output_duty_cycle_u = 0.0f;
    output_duty_cycle_v = 0.0f;
    output_duty_cycle_w = 0.0f;

    update_output_pwms();

    mode = Mode::IDLE;
}

void Executor::start_test_pwm(float duty_cycle_u, float duty_cycle_v,
                              float duty_cycle_w) {
    if (mode != Mode::IDLE) {
        return;
    }

    mode = Mode::TEST_PWM;

    motor_driver.turn_on();

    set_duty_cycle_u(duty_cycle_u);
    set_duty_cycle_v(duty_cycle_v);
    set_duty_cycle_w(duty_cycle_w);

    update_output_pwms();
}

void Executor::refresh_test_pwm() {
    if (mode != Mode::TEST_PWM) {
        return;
    }

    update_output_pwms();
}

void Executor::start_space_vector(float modulation_index,
                                  float modulation_frequency_hz) {
    if (mode != Mode::IDLE && mode != Mode::TEST_PWM) {
        return;
    }

    mode = Mode::SPACE_VECTOR;

    set_modulation_index(modulation_index);
    set_modulation_frequency_hz(modulation_frequency_hz);
    control_time = 0.0f;

    control_alarm_id =
        Time::register_mid_precision_alarm(space_vector_period_us, [&]() {
            control_time += space_vector_period_us / 1e6f;

            double sin_u{
                this->modulation_index *
                sin(2.0 * M_PI * this->modulation_frequency_hz * control_time)};
            double sin_v{
                this->modulation_index *
                sin(2.0 * M_PI * this->modulation_frequency_hz * control_time -
                    2.0 * M_PI / 3.0)};
            double sin_w{
                this->modulation_index *
                sin(2.0 * M_PI * this->modulation_frequency_hz * control_time +
                    2.0 * M_PI / 3.0)};

            double offset{(std::max({sin_u, sin_v, sin_w}) +
                           std::min({sin_u, sin_v, sin_w})) /
                          2.0};

            set_duty_cycle_u(100.0 * (sin_u - offset + 1.0) / 2.0);
            set_duty_cycle_v(100.0 * (sin_v - offset + 1.0) / 2.0);
            set_duty_cycle_w(100.0 * (sin_w - offset + 1.0) / 2.0);
            update_output_pwms();
        });
}

void Executor::start_emulated_current_control(float linear_speed,
                                              float d_current,
                                              float q_current) {
    if (mode != Mode::IDLE && mode != Mode::TEST_PWM) {
        return;
    }

    mode = Mode::EMULATED_CURRENT_CONTROL;

    current_controller.reset();

    set_linear_speed(linear_speed);
    set_d_current(d_current);
    set_q_current(q_current);
    control_time = 0.0;

    control_alarm_id =
        Time::register_mid_precision_alarm(current_control_period_us, [&]() {
            control_time += current_control_period_us / 1e6f;

            double position = this->linear_speed * control_time / 0.096;
            electrical_angle =
                M_PI * (position - (2.0 * floor(position / 2.0))) - 0.9367;

            update_current_control();
        });
}

void Executor::start_speed_control(float linear_speed) {
    // mode != Mode::CurrentControl is for backward compatibility to allow to go
    // from current control to speed control
    if (mode != Mode::IDLE && mode != Mode::TEST_PWM &&
        mode != Mode::CURRENT_CONTROL) {
        return;
    }

    mode = Mode::SPEED_CONTROL;

    set_linear_speed(linear_speed);

    control_alarm_id = Time::register_mid_precision_alarm(
        speed_control_period_us, [&]() { update_speed_control(); });
}

void Executor::start_current_control(float d_current, float q_current) {
    if (mode != Mode::IDLE && mode != Mode::TEST_PWM &&
        mode != Mode::SPEED_CONTROL) {
        return;
    }
    if (mode == Mode::SPEED_CONTROL) {
    } else {
        mode = Mode::CURRENT_CONTROL;
        set_q_current(q_current);
    }

    current_controller.reset();

    set_d_current(d_current);

    control_alarm_id =
        Time::register_mid_precision_alarm(current_control_period_us, [&]() {
            double position = (*speetec.get_position_ptr() - (0.0)) / 0.096;

            electrical_angle =
                M_PI * (position - (2.0 * floor(position / 2.0)));
            if (mode == Mode::SPEED_CONTROL) {
                set_q_current(*speed_controller.get_saturated_pi_output_ptr());
            } else {
                set_q_current(q_current_ref);
            }
            update_current_control();
        });
}

void Executor::update_current_control() {
    measured_u_current = *current_sense.get_average_phase_u_current();
    measured_v_current = *current_sense.get_average_phase_v_current();
    measured_w_current = *current_sense.get_average_phase_w_current();

    clarke_transform();
    park_transform();
    // combined_park_clarke_transformation();

    d_current_error = d_current_ref - measured_d_current;
    q_current_error = q_current_ref - measured_q_current;
    current_controller.update(d_current_error, q_current_error);
    target_d_voltage = current_controller.get_d_output();
    target_q_voltage = current_controller.get_q_output();

    //    combined_inverse_park_clarke_transform();
    inverse_park_transform();
    inverse_clarke_transform();

    double offset{
        (std::max({target_u_voltage, target_v_voltage, target_w_voltage}) +
         std::min({target_u_voltage, target_v_voltage, target_w_voltage})) /
        2.0};

    output_u_voltage = target_u_voltage - offset;
    output_v_voltage = target_v_voltage - offset;
    output_w_voltage = target_w_voltage - offset;

    float dc_link_voltage{*voltage_sense.get_dc_link_average_voltage_ptr()};

    set_duty_cycle_u(100.0 * (output_u_voltage / dc_link_voltage + 1.0) / 2.0);
    set_duty_cycle_v(100.0 * (output_v_voltage / dc_link_voltage + 1.0) / 2.0);
    set_duty_cycle_w(100.0 * (output_w_voltage / dc_link_voltage + 1.0) / 2.0);

    update_output_pwms();
}
void Executor::update_speed_control() {
    speed_controller.update(linear_speed - (float)*speetec.get_speed_ptr());
}
void Executor::clarke_transform() {
    measured_alpha_current =
        (2.0 * measured_u_current - measured_v_current - measured_w_current) /
        3.0;
    measured_beta_current =
        M_SQRT3 * (measured_v_current - measured_w_current) / 3.0;
    measured_zero_current =
        (measured_u_current + measured_v_current + measured_w_current) / 3.0;
}

void Executor::park_transform() {
    double offset{0.0};
    double sin_electrical_angle{sin(electrical_angle - offset)};
    double cos_electrical_angle{cos(electrical_angle - offset)};

    measured_d_current = cos_electrical_angle * measured_alpha_current +
                         sin_electrical_angle * measured_beta_current;
    measured_q_current = -sin_electrical_angle * measured_alpha_current +
                         cos_electrical_angle * measured_beta_current;
}

void Executor::combined_park_clarke_transformation() {
    measured_d_current =
        (measured_u_current * 2 / 3 * sin(electrical_angle) +
         (measured_v_current * 2 / 3 * sin(electrical_angle - 2 * M_PI / 3)) +
         +measured_w_current * 2 / 3 * sin(electrical_angle + 2 * M_PI / 3));
    measured_q_current =
        (measured_u_current * cos(electrical_angle) +
         (measured_v_current * cos(electrical_angle - 2 * M_PI / 3)) +
         measured_w_current * cos(electrical_angle + 2 * M_PI / 3));
}

void Executor::inverse_park_transform() {
    double offset{0.0};
    double sin_electrical_angle{sin(electrical_angle - offset)};
    double cos_electrical_angle{cos(electrical_angle - offset)};

    target_alpha_voltage = sin_electrical_angle * target_d_voltage +
                           cos_electrical_angle * target_q_voltage;
    target_beta_voltage = sin_electrical_angle * target_q_voltage -
                          cos_electrical_angle * target_d_voltage;
}

void Executor::inverse_clarke_transform() {
    double target_zero_voltage{0.0};  // assuming balanced tree phase system

    target_u_voltage = target_alpha_voltage + target_zero_voltage;
    target_v_voltage = M_SQRT3 * target_beta_voltage / 2.0 -
                       target_alpha_voltage / 2.0 + target_zero_voltage;
    target_w_voltage = target_zero_voltage - target_alpha_voltage / 2.0 -
                       M_SQRT3 * target_beta_voltage / 2.0;
}

void Executor::combined_inverse_park_clarke_transform() {
    target_u_voltage = target_d_voltage * cos(electrical_angle) -
                       target_q_voltage * sin(electrical_angle);
    target_v_voltage = target_d_voltage * cos(electrical_angle - 2 * M_PI / 3) -
                       target_q_voltage * sin(electrical_angle - 2 * M_PI / 3);
    target_w_voltage = target_d_voltage * cos(electrical_angle + 2 * M_PI / 3) -
                       target_q_voltage * sin(electrical_angle + 2 * M_PI / 3);
}

void Executor::update_output_pwms() {
    motor_driver.set_duty_cycle_u(output_duty_cycle_u);
    motor_driver.set_duty_cycle_v(output_duty_cycle_v);
    motor_driver.set_duty_cycle_w(output_duty_cycle_w);
}

void Executor::set_duty_cycle_u(float duty_cycle_u) {
    output_duty_cycle_u = duty_cycle_u;
}

void Executor::set_duty_cycle_v(float duty_cycle_v) {
    output_duty_cycle_v = duty_cycle_v;
}

void Executor::set_duty_cycle_w(float duty_cycle_w) {
    output_duty_cycle_w = duty_cycle_w;
}

void Executor::set_modulation_index(float modulation_index) {
    if (mode != Mode::SPACE_VECTOR) {
        return;
    }

    this->modulation_index = modulation_index;
}

void Executor::set_modulation_frequency_hz(float modulation_frequency_hz) {
    if (mode != Mode::SPACE_VECTOR) {
        return;
    }

    this->modulation_frequency_hz = modulation_frequency_hz;
}

void Executor::set_linear_speed(float linear_speed) {
    if (mode != Mode::SPEED_CONTROL) {
        return;
    }

    this->linear_speed = linear_speed;
}

void Executor::set_d_current(float d_current) {
    if (mode != Mode::EMULATED_CURRENT_CONTROL &&
        mode != Mode::CURRENT_CONTROL && mode != Mode::SPEED_CONTROL) {
        return;
    }

    this->d_current_ref = d_current;
}

void Executor::set_q_current(float q_current) {
    if (mode != Mode::EMULATED_CURRENT_CONTROL &&
        mode != Mode::CURRENT_CONTROL && mode != Mode::SPEED_CONTROL) {
        return;
    }

    this->q_current_ref = q_current;
}

float *Executor::get_space_vector_time() { return &control_time; }
float *Executor::get_modulation_index() { return &modulation_index; }
float *Executor::get_modulation_frequency_hz() {
    return &modulation_frequency_hz;
}

double *Executor::get_d_current_reference() { return &d_current_ref; }
double *Executor::get_measured_d_current() { return &measured_d_current; }
double *Executor::get_d_current_error() { return &d_current_error; }
double *Executor::get_q_current_reference() { return &q_current_ref; }
double *Executor::get_measured_q_current() { return &measured_q_current; }
double *Executor::get_q_current_error() { return &q_current_error; }
double *Executor::get_target_u_voltage() { return &target_u_voltage; }
double *Executor::get_target_v_voltage() { return &target_v_voltage; }
double *Executor::get_target_w_voltage() { return &target_w_voltage; }
double *Executor::get_electrical_angle() { return &electrical_angle; }
float *Executor::get_target_linear_speed() { return &target_linear_speed; };
float *Executor::get_saturated_pi_output_ptr() {
    return speed_controller.get_saturated_pi_output_ptr();
}
}  // namespace BCU::Control