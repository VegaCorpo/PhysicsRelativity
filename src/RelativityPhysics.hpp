#pragma once

#include <interfaces/IPhysicsEngine.hpp>

#include "components/RelativityState.hpp"
#include "integration/TimeStep.hpp"
#include "types/types.hpp"

namespace physics {
    class RelativityPhysics : public common::IPhysicsEngine {
        public:
            RelativityPhysics() = default;
            ~RelativityPhysics() override = default;

            void init(common::SpecificDataPhysics world) override;

            void update(double dt) override;

            void physicsPause() override { this->_state = common::ModuleState::PAUSE; };

            void physicsResume() override { this->_state = common::ModuleState::RUNNING; };

            void shutdown() override;

            void syncIn(common::SpecificDataPhysics world) override;
            common::WorldState publish() override;

            [[nodiscard]] std::string getName() const override { return "RelativityPhysics"; }

        private:
            void _publishOrientations(common::WorldState& world, std::size_t count) const;

            common::SpecificDataPhysics _worldState;
            RelativisticState _relativisticState;
            integration::TimeStep _timeStep;
            common::ModuleState _state = common::ModuleState::RUNNING;
    };
} // namespace physics
