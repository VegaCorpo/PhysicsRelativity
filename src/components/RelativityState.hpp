#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>
#include <types/World.hpp>

namespace physics {
    struct RelativisticState {
            // Keep the same distance and time units as Physics: kilometres and seconds.
            static constexpr double SPEED_OF_LIGHT = 299'792.458;

            std::vector<double> posX;
            std::vector<double> posY;
            std::vector<double> posZ;
            std::vector<double> velocityX;
            std::vector<double> velocityY;
            std::vector<double> velocityZ;
            std::vector<double> scalarMass;
            std::vector<double> lorentzFactor;

            [[nodiscard]] std::size_t size() const noexcept { return this->_count; }

            void syncIn(const common::SpecificDataPhysics& world)
            {
                const std::size_t count = std::min(
                    {world.entitiesId.size(), world.positions.size(), world.velocities.size(), world.masses.size()});
                this->_resize(count);

                for (std::size_t i = 0; i < count; ++i) {
                    const auto& position = world.positions[i];
                    const auto& velocity = world.velocities[i];
                    posX[i] = position.x;
                    posY[i] = position.y;
                    posZ[i] = position.z;
                    velocityX[i] = velocity.x;
                    velocityY[i] = velocity.y;
                    velocityZ[i] = velocity.z;
                    scalarMass[i] = static_cast<double>(world.masses[i].mantissa) *
                        std::pow(10.0, world.masses[i].exponent);
                    lorentzFactor[i] = _lorentzFactor(velocity.x, velocity.y, velocity.z);
                }
            }

            void syncOut(common::SpecificDataPhysics& world) const
            {
                const std::size_t count = std::min({this->_count, world.positions.size(), world.velocities.size()});
                for (std::size_t i = 0; i < count; ++i) {
                    world.positions[i].x = posX[i];
                    world.positions[i].y = posY[i];
                    world.positions[i].z = posZ[i];
                    world.velocities[i].x = velocityX[i];
                    world.velocities[i].y = velocityY[i];
                    world.velocities[i].z = velocityZ[i];
                }
            }

        private:
            static double _lorentzFactor(double vx, double vy, double vz)
            {
                const double betaSquared = (vx * vx + vy * vy + vz * vz) /
                    (SPEED_OF_LIGHT * SPEED_OF_LIGHT);
                if (betaSquared == 1.0)
                    return std::numeric_limits<double>::infinity();
                if (!std::isfinite(betaSquared) || betaSquared > 1.0)
                    return std::numeric_limits<double>::quiet_NaN();
                return 1.0 / std::sqrt(1.0 - betaSquared);
            }

            void _resize(std::size_t count)
            {
                if (count == this->_count)
                    return;
                this->_count = count;
                posX.resize(count);
                posY.resize(count);
                posZ.resize(count);
                velocityX.resize(count);
                velocityY.resize(count);
                velocityZ.resize(count);
                scalarMass.resize(count);
                lorentzFactor.resize(count);
            }

            std::size_t _count = 0;
    };
} // namespace physics
