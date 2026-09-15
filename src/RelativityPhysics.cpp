#include "RelativityPhysics.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>

void physics::RelativityPhysics::init(common::SpecificDataPhysics world)
{
    this->_worldState = std::move(world);
}

void physics::RelativityPhysics::update(double dt)
{
    (void)dt;
}

void physics::RelativityPhysics::shutdown()
{}

void physics::RelativityPhysics::syncIn(common::SpecificDataPhysics world)
{
    this->_worldState = std::move(world);
}

common::WorldState physics::RelativityPhysics::publish()
{
    const auto& src = this->_worldState;
    const std::size_t count = std::min({src.entitiesId.size(), src.positions.size(),
                                        src.velocities.size(), src.accelerations.size()});

    common::WorldState world;
    world.entitiesId.assign(src.entitiesId.begin(), src.entitiesId.begin() + count);
    world.positions.assign(src.positions.begin(), src.positions.begin() + count);
    world.velocities.assign(src.velocities.begin(), src.velocities.begin() + count);
    world.accelerations.assign(src.accelerations.begin(), src.accelerations.begin() + count);
    return world;
}
