#include "integration/Rk4.hpp"

#include <cstddef>
#include <utils/vector3.hpp>

void physics::integration::Rk4::step(const Bodies& bodies, IAccelerationField& field, double duration) const
{
    _saveInitialState(bodies);
    for (const Stage& stage : INTERMEDIATE_STAGES) {
        _evaluateStage(bodies, field, stage.weight);
        _moveToStage(bodies, stage.nextOffset * duration);
    }
    _evaluateStage(bodies, field, FINAL_STAGE_WEIGHT);
    _combineStages(bodies, duration);
}

void physics::integration::Rk4::_saveInitialState(const Bodies& bodies)
{
    const std::size_t count = bodies.state.size();

    bodies.workspace.initialPositions = bodies.state.positions;
    bodies.workspace.initialVelocities = bodies.state.velocities;
    bodies.workspace.positionIncrements.assign(count, common::math::Vector3{});
    bodies.workspace.velocityIncrements.assign(count, common::math::Vector3{});
}

void physics::integration::Rk4::_evaluateStage(const Bodies& bodies, IAccelerationField& field, double weight)
{
    RelativisticState& state = bodies.state;
    IntegrationWorkspace& workspace = bodies.workspace;

    field.compute(state);
    for (std::size_t body = 0; body < state.size(); body += 1) {
        workspace.positionIncrements[body] += state.velocities[body] * weight;
        workspace.velocityIncrements[body] += state.accelerations[body] * weight;
    }
}

void physics::integration::Rk4::_moveToStage(const Bodies& bodies, double offset)
{
    RelativisticState& state = bodies.state;
    const IntegrationWorkspace& workspace = bodies.workspace;

    for (std::size_t body = 0; body < state.size(); body += 1) {
        state.positions[body] = workspace.initialPositions[body] + state.velocities[body] * offset;
        state.velocities[body] = workspace.initialVelocities[body] + state.accelerations[body] * offset;
    }
}

void physics::integration::Rk4::_combineStages(const Bodies& bodies, double duration)
{
    RelativisticState& state = bodies.state;
    const IntegrationWorkspace& workspace = bodies.workspace;
    const double stageScale = duration / STAGE_WEIGHT_SUM;

    for (std::size_t body = 0; body < state.size(); body += 1) {
        state.positions[body] = workspace.initialPositions[body] + workspace.positionIncrements[body] * stageScale;
        state.velocities[body] = workspace.initialVelocities[body] + workspace.velocityIncrements[body] * stageScale;
    }
}
