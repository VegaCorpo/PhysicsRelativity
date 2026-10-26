#include "RelativityUtils.hpp"
#include "components/RelativityState.hpp"

double physics::utils::properTime(double coordinateTime, double speed)
{
    return coordinateTime / RelativisticState::lorentzFactorFor(speed, 0.0, 0.0);
}

double physics::utils::minkowskiIntervalSquared(double deltaTime, double deltaX, double deltaY, double deltaZ)
{
    const double cDeltaTime = RelativisticState::SPEED_OF_LIGHT * deltaTime;
    return cDeltaTime * cDeltaTime - deltaX * deltaX - deltaY * deltaY - deltaZ * deltaZ;
}
