#include "RelativityPhysics.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>
#include "utils/rotation.hpp"

//? Public methods

void physics::RelativityPhysics::init(common::SpecificDataPhysics world)
{
    this->_worldState = std::move(world);
    this->_relativisticState.syncIn(this->_worldState);
}

void physics::RelativityPhysics::update(double dt)
{
    if (this->_state != common::ModuleState::RUNNING)
        return;
    this->_relativisticState.syncIn(this->_worldState);
    common::rotation::advanceAll(this->_worldState.orientations, this->_worldState.angularVelocities, dt);
    this->_relativisticState.syncOut(this->_worldState);
}

void physics::RelativityPhysics::shutdown()
{}

//? Private methods

void physics::RelativityPhysics::syncIn(common::SpecificDataPhysics world)
{
    this->_worldState = std::move(world);
}

common::WorldState physics::RelativityPhysics::publish()
{
    common::WorldState world;
    const std::size_t count = std::min({this->_worldState.entitiesId.size(), this->_worldState.positions.size(),
                                        this->_worldState.velocities.size(), this->_worldState.accelerations.size()});

    world.entitiesId.resize(count);
    world.positions.resize(count);
    world.velocities.resize(count);
    world.accelerations.resize(count);

    for (std::size_t i = 0; i < count; ++i) {
        world.entitiesId[i] = this->_worldState.entitiesId[i];
        world.positions[i] = this->_worldState.positions[i];
        world.velocities[i] = this->_worldState.velocities[i];
        world.accelerations[i] = this->_worldState.accelerations[i];
    }
    this->_publishOrientations(world, count);
    return world;
}

void physics::RelativityPhysics::_publishOrientations(common::WorldState& world, std::size_t count) const
{
    const auto& orientations = this->_worldState.orientations;

    world.orientations.resize(count);
    std::copy_n(orientations.begin(), std::min(count, orientations.size()), world.orientations.begin());
}
