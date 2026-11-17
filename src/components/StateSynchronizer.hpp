#pragma once

#include <types/World.hpp>
#include "components/RelativisticState.hpp"

namespace physics::synchronization {
    void syncIn(const common::SpecificDataPhysics& world, RelativisticState& state);
    void syncOut(const RelativisticState& state, common::SpecificDataPhysics& world);
} // namespace physics::synchronization
