#pragma once

namespace physics::utils {
    inline constexpr double GRAVITATIONAL_CONSTANT = 6.67430e-20; // km^3 kg^-1 s^-2

    /** @brief Convert coordinate time to proper time for a constant speed (km/s). */
    [[nodiscard]] double properTime(double coordinateTime, double speed);

    /** @brief Minkowski interval squared using the (+---) sign convention (km²). */
    [[nodiscard]] double minkowskiIntervalSquared(double deltaTime, double deltaX, double deltaY, double deltaZ);
} // namespace physics::utils
