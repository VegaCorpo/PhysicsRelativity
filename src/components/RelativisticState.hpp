#pragma once

#include <cstddef>
#include <utils/vector3.hpp>
#include <vector>

namespace physics {
    struct RelativisticState {
            std::vector<std::size_t> entitiesId;
            std::vector<double> masses;
            std::vector<common::math::Vector3> positions;
            std::vector<common::math::Vector3> velocities;
            std::vector<common::math::Vector3> accelerations;
            std::vector<double> lorentzFactors;
            std::vector<double> properTimes;
            double softeningLength = 0.0;

            [[nodiscard]] std::size_t size() const noexcept { return this->entitiesId.size(); }

            void resize(std::size_t count)
            {
                this->entitiesId.resize(count);
                this->masses.resize(count);
                this->positions.resize(count);
                this->velocities.resize(count);
                this->accelerations.resize(count);
                this->lorentzFactors.resize(count);
                this->properTimes.resize(count);
            }
    };
} // namespace physics
