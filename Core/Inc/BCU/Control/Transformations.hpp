#pragma once

#include "math.h"

namespace BCU::Control {

struct ThreePhaseSystem {
    double u;
    double v;
    double w;

    ThreePhaseSystem(double u, double v, double w);
    ThreePhaseSystem(TwoPhaseSystem two_phase);

    TwoPhaseSystem clarke_transform();
    RotatingTwoPhaseSystem clarke_park_transform(double electrical_angle);
};

struct TwoPhaseSystem {
    double alpha;
    double beta;
    double zero;

    TwoPhaseSystem(double alpha, double beta);
    TwoPhaseSystem(RotatingTwoPhaseSystem rotating_two_phase,
                   double electrical_angle);
    TwoPhaseSystem(ThreePhaseSystem three_phase);

    RotatingTwoPhaseSystem park_transform(double electrical_angle);
    ThreePhaseSystem inverse_clarke_transform();
};

struct RotatingTwoPhaseSystem {
    double d;
    double q;
    double zero;

    RotatingTwoPhaseSystem(double d, double q);
    RotatingTwoPhaseSystem(TwoPhaseSystem two_phase, double electrical_angle);

    TwoPhaseSystem inverse_park_transform(double electrical_angle);
    ThreePhaseSystem inverse_clarke_park_transform(double electrical_angle);
};

};  // namespace BCU::Control