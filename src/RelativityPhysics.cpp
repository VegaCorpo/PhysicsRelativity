#include "RelativityPhysics.hpp"

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

common::SpecificDataPhysics physics::RelativityPhysics::syncOut()
{
    return this->_worldState;
}
