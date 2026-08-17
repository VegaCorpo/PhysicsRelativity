#include "RelativityPhysics.hpp"

#include <utility>

void physics::RelativityPhysics::init(common::WorldState world)
{
    this->_worldState = std::move(world);
}

void physics::RelativityPhysics::update(double dt)
{
    (void)dt;
}

void physics::RelativityPhysics::shutdown()
{}

void physics::RelativityPhysics::syncIn(common::WorldState world)
{
    this->_worldState = std::move(world);
}

common::WorldState physics::RelativityPhysics::syncOut()
{
    return this->_worldState;
}
