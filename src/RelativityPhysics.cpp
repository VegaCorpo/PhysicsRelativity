#include "RelativityPhysics.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>
#include "components/StateSynchronizer.hpp"
#include "kinematics/ProperTime.hpp"
#include "utils/rotation.hpp"

namespace {
    template <typename Value>
    void copyPrefix(const std::vector<Value>& source, std::vector<Value>& destination, std::size_t count)
    {
        destination.resize(count);
        std::copy_n(source.begin(), std::min(count, source.size()), destination.begin());
    }
} // namespace

physics::RelativityPhysics::RelativityPhysics(std::unique_ptr<integration::IAccelerationField> field,
                                              std::unique_ptr<integration::IIntegrator> integrator) :
    _field(std::move(field)), _integrator(std::move(integrator)), _timeStep(_timeStepOf(*this->_integrator))
{}

void physics::RelativityPhysics::init(common::SpecificDataPhysics world)
{
    this->_worldState = std::move(world);
    synchronization::syncIn(this->_worldState, this->_relativisticState);
    kinematics::refreshLorentzFactors(this->_relativisticState);
}

void physics::RelativityPhysics::update(double dt)
{
    const std::expected<common::integration::SubSteps, common::integration::TimeStepError> subSteps =
        this->_timeStep.split(dt);

    if (!this->_canAdvance(subSteps))
        return;
    synchronization::syncIn(this->_worldState, this->_relativisticState);
    this->_advance(*subSteps);
    common::rotation::advanceAll(this->_worldState.orientations, this->_worldState.angularVelocities, dt);
    synchronization::syncOut(this->_relativisticState, this->_worldState);
}

void physics::RelativityPhysics::shutdown()
{}

void physics::RelativityPhysics::syncIn(common::SpecificDataPhysics world)
{
    this->_worldState = std::move(world);
}

common::WorldState physics::RelativityPhysics::publish()
{
    common::WorldState world;
    const std::size_t count = this->_publishedCount();

    copyPrefix(this->_worldState.entitiesId, world.entitiesId, count);
    copyPrefix(this->_worldState.positions, world.positions, count);
    copyPrefix(this->_worldState.velocities, world.velocities, count);
    copyPrefix(this->_worldState.accelerations, world.accelerations, count);
    copyPrefix(this->_worldState.orientations, world.orientations, count);
    return world;
}

common::integration::TimeStep physics::RelativityPhysics::_timeStepOf(const integration::IIntegrator& integrator)
{
    return common::integration::TimeStep::forForceEvaluations(integrator.forceEvaluationsPerStep())
        .value_or(common::integration::TimeStep{});
}

bool physics::RelativityPhysics::_canAdvance(
    const std::expected<common::integration::SubSteps, common::integration::TimeStepError>& subSteps) const
{
    return this->_state == common::ModuleState::RUNNING && subSteps.has_value() && !subSteps->empty();
}

void physics::RelativityPhysics::_advance(const common::integration::SubSteps& subSteps)
{
    for (std::size_t stepIndex = 0; stepIndex < subSteps.count; stepIndex += 1)
        this->_advanceOneStep(subSteps.duration);
    this->_field->compute(this->_relativisticState);
    kinematics::refreshLorentzFactors(this->_relativisticState);
}

void physics::RelativityPhysics::_advanceOneStep(double duration)
{
    const integration::Bodies bodies{this->_relativisticState, this->_workspace};

    kinematics::refreshLorentzFactors(this->_relativisticState);
    kinematics::advanceProperTimes(this->_relativisticState, duration);
    this->_integrator->step(bodies, *this->_field, duration);
}

std::size_t physics::RelativityPhysics::_publishedCount() const
{
    return std::min({this->_worldState.entitiesId.size(), this->_worldState.positions.size(),
                     this->_worldState.velocities.size(), this->_worldState.accelerations.size()});
}
