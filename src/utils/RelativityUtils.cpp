#include "RelativityUtils.hpp"
#include "components/RelativityState.hpp"

#include <cmath>

namespace {

    [[nodiscard]] bool isFinite(const physics::utils::RelativeVelocity& velocity)
    {
        return std::isfinite(velocity.x) && std::isfinite(velocity.y) && std::isfinite(velocity.z);
    }

    [[nodiscard]] long double speedOfLight()
    {
        return static_cast<long double>(physics::RelativisticState::SPEED_OF_LIGHT);
    }

} // namespace

double physics::utils::properTime(double coordinateTime, double speed)
{
    return coordinateTime / RelativisticState::lorentzFactorFor(speed, 0.0, 0.0);
}

double physics::utils::minkowskiIntervalSquared(double deltaTime, double deltaX, double deltaY, double deltaZ)
{
    const double cDeltaTime = RelativisticState::SPEED_OF_LIGHT * deltaTime;
    return cDeltaTime * cDeltaTime - deltaX * deltaX - deltaY * deltaY - deltaZ * deltaZ;
}

double physics::utils::speedMagnitude(const RelativeVelocity& velocity) noexcept
{
    return std::hypot(velocity.x, velocity.y, velocity.z);
}

std::expected<double, physics::utils::LorentzTransformError>
physics::utils::lorentzFactor(const RelativeVelocity& velocity)
{
    if (!isFinite(velocity))
        return std::unexpected(LorentzTransformError::NON_FINITE_INPUT);

    const double speed = speedMagnitude(velocity);
    const double c = RelativisticState::SPEED_OF_LIGHT;
    if (!std::isfinite(speed) || speed >= c)
        return std::unexpected(LorentzTransformError::SPEED_NOT_SUBLUMINAL);

    const double beta = speed / c;
    return 1.0 / std::sqrt((1.0 - beta) * (1.0 + beta));
}

std::expected<physics::utils::RelativeVelocity, physics::utils::LorentzTransformError>
physics::utils::transformVelocityToMovingFrame(const RelativeVelocity& velocity, const RelativeVelocity& frameVelocity)
{
    const std::expected<double, LorentzTransformError> gammaResult = lorentzFactor(frameVelocity);
    if (!gammaResult)
        return std::unexpected(gammaResult.error());
    const double frameSpeedValue = speedMagnitude(frameVelocity);
    if (!isFinite(velocity))
        return std::unexpected(LorentzTransformError::NON_FINITE_INPUT);
    const double objectSpeed = speedMagnitude(velocity);
    if (!std::isfinite(objectSpeed) || objectSpeed >= RelativisticState::SPEED_OF_LIGHT)
        return std::unexpected(LorentzTransformError::SPEED_NOT_SUBLUMINAL);

    const long double c = speedOfLight();
    const long double vx = frameVelocity.x;
    const long double vy = frameVelocity.y;
    const long double vz = frameVelocity.z;
    const long double ux = velocity.x;
    const long double uy = velocity.y;
    const long double uz = velocity.z;
    const long double frameSpeed = frameSpeedValue;
    if (frameSpeed == 0.0L)
        return velocity;
    const long double nx = vx / frameSpeed;
    const long double ny = vy / frameSpeed;
    const long double nz = vz / frameSpeed;

    const long double velocityAlongBoost = ux * nx + uy * ny + uz * nz;
    const long double dotOverCSquared = velocityAlongBoost * frameSpeed / (c * c);
    const long double gamma = *gammaResult;
    const long double denominator = gamma * (1.0L - dotOverCSquared);
    const long double parallelCorrection = (gamma - 1.0L) * velocityAlongBoost - gamma * frameSpeed;
    const long double resultX = (ux + parallelCorrection * nx) / denominator;
    const long double resultY = (uy + parallelCorrection * ny) / denominator;
    const long double resultZ = (uz + parallelCorrection * nz) / denominator;

    RelativeVelocity result{static_cast<double>(resultX), static_cast<double>(resultY), static_cast<double>(resultZ)};
    if (!isFinite(result))
        return std::unexpected(LorentzTransformError::NON_FINITE_RESULT);

    // Rounding can put a mathematically subluminal result one ULP at or above c.
    const double resultSpeed = speedMagnitude(result);
    if (resultSpeed >= RelativisticState::SPEED_OF_LIGHT) {
        const double largestRepresentableSubluminalSpeed = std::nextafter(RelativisticState::SPEED_OF_LIGHT, 0.0);
        const double scale = largestRepresentableSubluminalSpeed / resultSpeed;
        result.x *= scale;
        result.y *= scale;
        result.z *= scale;
    }
    return result;
}

std::expected<physics::utils::SpacetimeEvent, physics::utils::LorentzTransformError>
physics::utils::transformToMovingFrame(const physics::utils::SpacetimeEvent& event,
                                       const physics::utils::RelativeVelocity& frameVelocity)
{
    if (!std::isfinite(event.time) || !std::isfinite(event.x) || !std::isfinite(event.y) || !std::isfinite(event.z) ||
        !std::isfinite(frameVelocity.x) || !std::isfinite(frameVelocity.y) || !std::isfinite(frameVelocity.z))
        return std::unexpected(LorentzTransformError::NON_FINITE_INPUT);

    const double speed = speedMagnitude(frameVelocity);
    const double c = RelativisticState::SPEED_OF_LIGHT;
    if (!std::isfinite(speed) || speed >= c)
        return std::unexpected(LorentzTransformError::SPEED_NOT_SUBLUMINAL);
    if (speed == 0.0)
        return event;

    const long double vx = frameVelocity.x;
    const long double vy = frameVelocity.y;
    const long double vz = frameVelocity.z;
    const long double x = event.x;
    const long double y = event.y;
    const long double z = event.z;
    const long double time = event.time;
    const long double frameSpeed = speed;
    const long double nx = vx / frameSpeed;
    const long double ny = vy / frameSpeed;
    const long double nz = vz / frameSpeed;
    const long double cLong = speedOfLight();
    const long double beta = static_cast<long double>(speed) / cLong;
    const long double gamma = 1.0L / std::sqrt((1.0L - beta) * (1.0L + beta));
    const long double positionAlongBoost = nx * x + ny * y + nz * z;
    const long double spatialCorrection = (gamma - 1.0L) * positionAlongBoost - gamma * time * frameSpeed;

    const SpacetimeEvent transformed{
        static_cast<double>(gamma * (time - frameSpeed * positionAlongBoost / (cLong * cLong))),
        static_cast<double>(x + spatialCorrection * nx),
        static_cast<double>(y + spatialCorrection * ny),
        static_cast<double>(z + spatialCorrection * nz),
    };
    if (!std::isfinite(transformed.time) || !std::isfinite(transformed.x) || !std::isfinite(transformed.y) ||
        !std::isfinite(transformed.z))
        return std::unexpected(LorentzTransformError::NON_FINITE_RESULT);
    return transformed;
}

std::expected<physics::utils::SpacetimeEvent, physics::utils::LorentzTransformError>
physics::utils::rotateSpatially(const physics::utils::SpacetimeEvent& event, double angleRadians,
                                const physics::utils::RelativeVelocity& axis)
{
    if (!std::isfinite(event.time) || !std::isfinite(event.x) || !std::isfinite(event.y) || !std::isfinite(event.z) ||
        !std::isfinite(angleRadians) || !isFinite(axis))
        return std::unexpected(LorentzTransformError::NON_FINITE_INPUT);

    const double norm = speedMagnitude(axis);
    if (!(norm > 0.0))
        return std::unexpected(LorentzTransformError::NON_FINITE_INPUT);

    const long double nx = axis.x / norm, ny = axis.y / norm, nz = axis.z / norm;
    const long double c = std::cos(static_cast<long double>(angleRadians));
    const long double s = std::sin(static_cast<long double>(angleRadians));
    const long double d = nx * event.x + ny * event.y + nz * event.z;
    const long double k = 1.0L - c;

    SpacetimeEvent result{event.time, static_cast<double>(event.x * c + (ny * event.z - nz * event.y) * s + nx * d * k),
                          static_cast<double>(event.y * c + (nz * event.x - nx * event.z) * s + ny * d * k),
                          static_cast<double>(event.z * c + (nx * event.y - ny * event.x) * s + nz * d * k)};
    if (!std::isfinite(result.x) || !std::isfinite(result.y) || !std::isfinite(result.z))
        return std::unexpected(LorentzTransformError::NON_FINITE_RESULT);
    return result;
}

std::expected<physics::utils::SpacetimeEvent, physics::utils::LorentzTransformError>
physics::utils::transformFromMovingFrame(const physics::utils::SpacetimeEvent& event,
                                         const physics::utils::RelativeVelocity& frameVelocity)
{
    return transformToMovingFrame(event, {-frameVelocity.x, -frameVelocity.y, -frameVelocity.z});
}

std::expected<physics::utils::SpacetimeEvent, physics::utils::LorentzTransformError>
physics::utils::transformBetweenFrames(const physics::utils::SpacetimeEvent& event,
                                       const physics::utils::RelativeVelocity& firstFrameVelocity,
                                       const physics::utils::RelativeVelocity& secondFrameVelocity)
{
    auto intermediate = transformToMovingFrame(event, firstFrameVelocity);
    if (!intermediate)
        return std::unexpected(intermediate.error());
    return transformToMovingFrame(*intermediate, secondFrameVelocity);
}
