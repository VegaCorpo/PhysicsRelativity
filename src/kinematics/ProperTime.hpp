#pragma once

#include "components/RelativisticState.hpp"

namespace physics::kinematics {
    void refreshLorentzFactors(RelativisticState& state);
    void advanceProperTimes(RelativisticState& state, double coordinateDuration);
} // namespace physics::kinematics
