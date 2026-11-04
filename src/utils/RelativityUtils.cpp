#include "RelativityUtils.hpp"
#include "components/RelativityState.hpp"

#include <cmath>

double physics::utils::properTime(double coordinateTime, double speed)
{
    return coordinateTime / RelativisticState::lorentzFactorFor(speed, 0.0, 0.0);
}

double physics::utils::minkowskiIntervalSquared(double deltaTime, double deltaX, double deltaY, double deltaZ)
{
    const double cDeltaTime = RelativisticState::SPEED_OF_LIGHT * deltaTime;
    return cDeltaTime * cDeltaTime - deltaX * deltaX - deltaY * deltaY - deltaZ * deltaZ;
}

std::expected<physics::utils::SpacetimeEvent, physics::utils::LorentzTransformError>
physics::utils::transformToMovingFrame(const physics::utils::SpacetimeEvent& event,
                                       const physics::utils::RelativeVelocity& frameVelocity)
{
    if (!std::isfinite(event.time) || !std::isfinite(event.x) || !std::isfinite(event.y) || !std::isfinite(event.z) ||
        !std::isfinite(frameVelocity.x) || !std::isfinite(frameVelocity.y) || !std::isfinite(frameVelocity.z))
        return std::unexpected(LorentzTransformError::NON_FINITE_INPUT);

    const double speedSquared = frameVelocity.x * frameVelocity.x + frameVelocity.y * frameVelocity.y +
                                frameVelocity.z * frameVelocity.z;
    const double lightSpeedSquared = RelativisticState::SPEED_OF_LIGHT * RelativisticState::SPEED_OF_LIGHT;
    if (!std::isfinite(speedSquared) || speedSquared >= lightSpeedSquared)
        return std::unexpected(LorentzTransformError::SPEED_NOT_SUBLUMINAL);
    if (speedSquared == 0.0)
        return event;

    const double gamma = 1.0 / std::sqrt(1.0 - speedSquared / lightSpeedSquared);
    const double velocityDotPosition =
        frameVelocity.x * event.x + frameVelocity.y * event.y + frameVelocity.z * event.z;
    const double spatialFactor = ((gamma - 1.0) * velocityDotPosition / speedSquared) - gamma * event.time;

    const SpacetimeEvent transformed{
        gamma * (event.time - velocityDotPosition / lightSpeedSquared),
        event.x + spatialFactor * frameVelocity.x,
        event.y + spatialFactor * frameVelocity.y,
        event.z + spatialFactor * frameVelocity.z,
    };
    if (!std::isfinite(transformed.time) || !std::isfinite(transformed.x) || !std::isfinite(transformed.y) ||
        !std::isfinite(transformed.z))
        return std::unexpected(LorentzTransformError::NON_FINITE_INPUT);
    return transformed;
}
