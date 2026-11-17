#include "kinematics/ProperTime.hpp"

#include <algorithm>
#include <limits>
#include <utils/vector3.hpp>
#include "utils/RelativityUtils.hpp"

namespace {
    constexpr double INVALID_LORENTZ_FACTOR = std::numeric_limits<double>::quiet_NaN();

    [[nodiscard]] double lorentzFactorOf(const common::math::Vector3& velocity)
    {
        return physics::utils::lorentzFactor({velocity.x, velocity.y, velocity.z}).value_or(INVALID_LORENTZ_FACTOR);
    }
} // namespace

void physics::kinematics::refreshLorentzFactors(RelativisticState& state)
{
    std::ranges::transform(state.velocities, state.lorentzFactors.begin(), lorentzFactorOf);
}

void physics::kinematics::advanceProperTimes(RelativisticState& state, double coordinateDuration)
{
    std::ranges::transform(state.properTimes, state.lorentzFactors, state.properTimes.begin(),
                           [coordinateDuration](double properTime, double lorentzFactor)
                           { return properTime + coordinateDuration / lorentzFactor; });
}
