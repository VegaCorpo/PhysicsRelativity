#include "TimeStep.hpp"

#include <cmath>

auto physics::integration::TimeStep::split(double frameDuration) const -> std::expected<SubSteps, TimeStepError>
{
    if (!std::isfinite(frameDuration))
        return std::unexpected(TimeStepError::NOT_FINITE);
    if (frameDuration < 0.0)
        return std::unexpected(TimeStepError::NEGATIVE);
    if (frameDuration == 0.0)
        return SubSteps{};

    const double count = std::ceil(frameDuration / DEFAULT_MAX_SUB_STEP);
    if (count > static_cast<double>(MAX_SUB_STEP_COUNT))
        return std::unexpected(TimeStepError::TOO_MANY_SUB_STEPS);
    return SubSteps{static_cast<std::size_t>(count), frameDuration / count};
}
