#pragma once

#include <math.h>

#include "BCU/Control/CurrentControl.hpp"
#include "BCU/Control/VelocityControl.hpp"

namespace BCU::Control {

class FOC {
    struct Input {
        float velocity;
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

    CurrentControl current_control;
    VelocityControl velocity_control;

   public:
    Output execute(const Input &in) {
        current_control.set_q_reference(velocity_control.execute(in.velocity));

        auto [u_out, v_out, w_out] = current_control.execute(
            {.position{in.position}, .u{in.u}, .v{in.v}, .w{in.w}});

        return {.u{u_out}, .v{v_out}, .w{w_out}};
    }
};

}  // namespace BCU::Control