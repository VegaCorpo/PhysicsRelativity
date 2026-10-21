#include <cmath>
#include <limits>

#include "RelativityUtils.hpp"
#include "components/RelativityState.hpp"

double physics::utils::properTime(double coordinateTime, double speed)
{
    const double beta = speed / RelativisticState::SPEED_OF_LIGHT;
    const double betaSquared = beta * beta;

    if (betaSquared == 1.0)
        return 0.0;
    if (!std::isfinite(betaSquared) || betaSquared > 1.0)
        return std::numeric_limits<double>::quiet_NaN();
    return coordinateTime * std::sqrt(1.0 - betaSquared);
}
