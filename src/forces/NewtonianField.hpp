#pragma once

#include <cstddef>
#include <utils/vector3.hpp>
#include "integration/Bodies.hpp"

namespace physics::forces {
    class NewtonianField : public integration::IAccelerationField {
        public:
            void compute(RelativisticState& state) override;

        private:
            [[nodiscard]] static common::math::Vector3 _accelerationOn(const RelativisticState& state,
                                                                       std::size_t target);
            [[nodiscard]] static common::math::Vector3 _pullOf(const RelativisticState& state, std::size_t source,
                                                               std::size_t target);
    };
} // namespace physics::forces
