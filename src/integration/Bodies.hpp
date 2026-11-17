#pragma once

#include <integration/IAccelerationField.hpp>
#include <integration/IIntegrator.hpp>
#include <utils/vector3.hpp>
#include <vector>
#include "components/RelativisticState.hpp"

namespace physics::integration {
    struct IntegrationWorkspace {
            std::vector<common::math::Vector3> initialPositions;
            std::vector<common::math::Vector3> initialVelocities;
            std::vector<common::math::Vector3> positionIncrements;
            std::vector<common::math::Vector3> velocityIncrements;
    };

    struct Bodies {
            RelativisticState& state;
            IntegrationWorkspace& workspace;
    };

    using IAccelerationField = common::integration::IAccelerationField<RelativisticState>;
    using IIntegrator = common::integration::IIntegrator<Bodies, RelativisticState>;
} // namespace physics::integration
