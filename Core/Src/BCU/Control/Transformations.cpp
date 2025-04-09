#include "BCU/Control/Transformations.hpp"

namespace BCU::Control {

ThreePhaseSystem::ThreePhaseSystem(double u, double v, double w)
    : u(u), v(v), w(w) {}

TwoPhaseSystem::TwoPhaseSystem(double alpha, double beta)
    : alpha(alpha), beta(beta), zero(0.0f) {}

RotatingTwoPhaseSystem::RotatingTwoPhaseSystem(double d, double q)
    : d(d), q(q), zero(0.0f) {}

// Clarke Transform
TwoPhaseSystem::TwoPhaseSystem(ThreePhaseSystem three_phase)
    : alpha((2.0f * three_phase.u / 3.0f) - (three_phase.v / 3.0f) -
            (three_phase.w / 3.0f)),
      beta((M_SQRT3 * three_phase.v / 3.0f) - (M_SQRT3 * three_phase.w / 3.0f)),
      zero((three_phase.u / 3.0f) + (three_phase.v / 3.0f) +
           (three_phase.w / 3.0f)) {}

// Park Transform (Q-Axis aligned)
RotatingTwoPhaseSystem::RotatingTwoPhaseSystem(TwoPhaseSystem two_phase,
                                               double electrical_angle)
    : d((two_phase.alpha * sin(electrical_angle)) -
        (two_phase.beta * cos(electrical_angle))),
      q((two_phase.alpha * cos(electrical_angle)) +
        (two_phase.beta * sin(electrical_angle))),
      zero(two_phase.zero) {}

// Inverse Park Transform (Q-Axis aligned)
TwoPhaseSystem::TwoPhaseSystem(RotatingTwoPhaseSystem rotating_two_phase,
                               double electrical_angle)
    : alpha((rotating_two_phase.d * sin(electrical_angle)) +
            (rotating_two_phase.q * cos(electrical_angle))),
      beta((rotating_two_phase.q * sin(electrical_angle)) -
           (rotating_two_phase.d * cos(electrical_angle))),
      zero(rotating_two_phase.zero) {}

// Inverse Clarke Transform
ThreePhaseSystem::ThreePhaseSystem(TwoPhaseSystem two_phase)
    : u(two_phase.alpha + two_phase.zero),
      v((M_SQRT3 * two_phase.beta / 2.0f) + two_phase.zero -
        (two_phase.alpha / 2.0f)),
      w(two_phase.zero - (two_phase.alpha / 2.0f) -
        (M_SQRT3 * two_phase.beta / 2.0f)) {}

TwoPhaseSystem ThreePhaseSystem::clarke_transform() {
    return TwoPhaseSystem(*this);
}

RotatingTwoPhaseSystem TwoPhaseSystem::park_transform(double electrical_angle) {
    return RotatingTwoPhaseSystem(*this, electrical_angle);
}

ThreePhaseSystem TwoPhaseSystem::inverse_clarke_transform() {
    return ThreePhaseSystem(*this);
}

TwoPhaseSystem RotatingTwoPhaseSystem::inverse_park_transform(
    double electrical_angle) {
    return TwoPhaseSystem(*this, electrical_angle);
}

ThreePhaseSystem RotatingTwoPhaseSystem::inverse_clarke_park_transform(
    double electrical_angle) {
    return ThreePhaseSystem(TwoPhaseSystem(*this, electrical_angle));
}

RotatingTwoPhaseSystem ThreePhaseSystem::clarke_park_transform(
    double electrical_angle) {
    return RotatingTwoPhaseSystem(TwoPhaseSystem(*this), electrical_angle);
}

}  // namespace BCU::Control
