#pragma once

#include <expected>

namespace physics::utils {
    inline constexpr double GRAVITATIONAL_CONSTANT = 6.67430e-20; // km^3 kg^-1 s^-2

    /** @brief Spacetime event in the engine units: seconds and kilometres. */
    struct SpacetimeEvent {
        double time = 0.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    /** @brief Three-dimensional relative velocity in km/s. */
    struct RelativeVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    enum class LorentzTransformError {
        NON_FINITE_INPUT,
        SPEED_NOT_SUBLUMINAL,
        NON_FINITE_RESULT
    };

    /** @brief Relativistic speed magnitude in km/s. */
    [[nodiscard]] double speedMagnitude(const RelativeVelocity& velocity) noexcept;

    /** @brief Checked Lorentz factor; rejects non-finite or luminal/superluminal speeds. */
    [[nodiscard]] std::expected<double, LorentzTransformError> lorentzFactor(const RelativeVelocity& velocity);

    /**
     * @brief Transform a velocity measured in S into S', which moves at frameVelocity relative to S.
     * This is the relativistic velocity composition law; all velocities are in km/s.
     */
    [[nodiscard]] std::expected<RelativeVelocity, LorentzTransformError>
    transformVelocityToMovingFrame(const RelativeVelocity& velocity, const RelativeVelocity& frameVelocity);

    /** @brief Convert coordinate time to proper time for a constant speed (km/s). */
    [[nodiscard]] double properTime(double coordinateTime, double speed);

    /** @brief Minkowski interval squared using the (+---) sign convention (km²). */
    [[nodiscard]] double minkowskiIntervalSquared(double deltaTime, double deltaX, double deltaY, double deltaZ);

    /**
     * @brief Express an event in S', where S' moves at @p frameVelocity relative to S.
     * Origins coincide at t = 0; uses the (+---) convention and km/s, seconds, kilometres.
     */
    [[nodiscard]] std::expected<SpacetimeEvent, LorentzTransformError>
    transformToMovingFrame(const SpacetimeEvent& event, const RelativeVelocity& frameVelocity);
} // namespace physics::utils
