#pragma once

#include "BCU/Control/Transformations/Clarke.hpp"
#include "BCU/Control/Transformations/InversePark.hpp"
#include "BCU/Control/Transformations/Park.hpp"
#include "ST-LIB_HIGH/Control/Blocks/PI.hpp"

namespace BCU::Control {

class CurrentControl {
    constexpr static float D_OUT_UPPER_LIMIT{300.0f};
    constexpr static float D_OUT_LOWER_LIMIT{-300.0f};

    constexpr static float Q_OUT_UPPER_LIMIT{300.0f};
    constexpr static float Q_OUT_LOWER_LIMIT{-300.0f};

    struct Input {
        float position;
        float u;
        float v;
        float w;
    };

    struct Output {
        float u;
        float v;
        float w;
    };

    float d_reference{0.0f};
    float q_reference{0.0f};

    PI<IntegratorType::Trapezoidal> d_pi;
    PI<IntegratorType::Trapezoidal> q_pi;

    float calculate_electrical_angle(const float &position) {
        float scaled_position{position / 0.096f};
        return (scaled_position - (std::floor(scaled_position / 2.0f) * 2.0f)) *
               M_PI;
    }

    inline float execute_d(const float &d) {
        d_pi.input_value = d_reference - d;

        d_pi.execute();

        float d_out{d_pi.output_value};
        if (d_out >= D_OUT_UPPER_LIMIT)
            d_out = D_OUT_UPPER_LIMIT;
        else if (d_out <= D_OUT_LOWER_LIMIT)
            d_out = D_OUT_LOWER_LIMIT;

        return d_out;
    }

    inline float execute_q(const float &q) {
        q_pi.input_value = q_reference - q;

        q_pi.execute();

        float q_out{q_pi.output_value};
        if (q_out >= Q_OUT_UPPER_LIMIT)
            q_out = Q_OUT_UPPER_LIMIT;
        else if (q_out <= Q_OUT_LOWER_LIMIT)
            q_out = Q_OUT_LOWER_LIMIT;

        return q_out;
    }

   public:
    CurrentControl(float kp_d, float ki_d, float period_d, float kp_q,
                   float ki_q, float period_q)
        : d_pi(kp_d, ki_d, period_d), q_pi(kp_q, ki_q, period_q) {}

    Output execute(const Input &in) {
        float electrical_angle{calculate_electrical_angle(in.position)};

        auto [alpha, beta, zero] = clarke_transform(in.u, in.v, in.w);

        auto [d, q] = park_transform(electrical_angle, alpha, beta);

        float d_out{execute_d(d)};
        float q_out{execute_q(q)};

        auto [u_out, v_out, w_out] =
            inverse_park_transform(electrical_angle, d_out, q_out);

        return {
            .u{u_out},
            .v{v_out},
            .w{w_out},
        };
    }

    void set_d_reference(float new_d) { d_reference = new_d; }

    void set_q_reference(float new_q) { q_reference = new_q; }

    void reset() {
        d_pi.reset();
        q_pi.reset();
    }

    void update_d_kp(float kp) {
        d_pi.set_kp(kp);
        d_pi.reset();
    }

    void update_d_ki(float ki) {
        d_pi.set_ki(ki);
        d_pi.reset();
    }

    void update_d_period(float period) {
        d_pi.set_period(period);
        d_pi.reset();
    }

    void update_q_kp(float kp) {
        q_pi.set_kp(kp);
        q_pi.reset();
    }

    void update_q_ki(float ki) {
        q_pi.set_ki(ki);
        q_pi.reset();
    }

    void update_q_period(float period) {
        q_pi.set_period(period);
        q_pi.reset();
    }
};

};  // namespace BCU::Control