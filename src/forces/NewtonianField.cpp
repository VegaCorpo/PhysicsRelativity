#include "forces/NewtonianField.hpp"

#include <cmath>
#include <utils/physicalConstants.hpp>

using common::math::Vector3;

void physics::forces::NewtonianField::compute(RelativisticState& state)
{
    for (std::size_t target = 0; target < state.size(); target += 1)
        state.accelerations[target] = _accelerationOn(state, target);
}

Vector3 physics::forces::NewtonianField::_accelerationOn(const RelativisticState& state, std::size_t target)
{
    Vector3 acceleration;

    for (std::size_t source = 0; source < state.size(); source += 1)
        acceleration += _pullOf(state, source, target);
    return acceleration;
}

Vector3 physics::forces::NewtonianField::_pullOf(const RelativisticState& state, std::size_t source, std::size_t target)
{
    if (source == target)
        return {};

    const Vector3 separation = state.positions[source] - state.positions[target];
    const double softenedDistance =
        std::sqrt(common::math::squaredNorm(separation) + state.softeningLength * state.softeningLength);
    const double strength = common::constants::GRAVITATIONAL_CONSTANT * state.masses[source] /
        (softenedDistance * softenedDistance * softenedDistance);

    return separation * strength;
}
