#pragma once

#include "math.h"

namespace BCU::Control {

struct ThreePhaseSystem {
    float u;
    float v;
    float w;

    ThreePhaseSystem(float u, float v, float w);
    ThreePhaseSystem(TwoPhaseSystem two_phase);

    TwoPhaseSystem clarke_transform();
};

struct TwoPhaseSystem {
    float alpha;
    float beta;
    float zero;

    TwoPhaseSystem(float alpha, float beta);
    TwoPhaseSystem(RotatingTwoPhaseSystem rotating_two_phase,
                   float electrical_angle);
    TwoPhaseSystem(ThreePhaseSystem three_phase);

    RotatingTwoPhaseSystem park_transform(float electrical_angle);
    ThreePhaseSystem inverse_clarke_transform();
};

struct RotatingTwoPhaseSystem {
    float d;
    float q;
    float zero;

    RotatingTwoPhaseSystem(float d, float q);
    RotatingTwoPhaseSystem(TwoPhaseSystem two_phase, float electrical_angle);

    TwoPhaseSystem inverse_park_transform(float electrical_angle);
};

};  // namespace BCU::Control