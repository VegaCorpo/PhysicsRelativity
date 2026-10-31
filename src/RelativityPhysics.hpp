#pragma once

#include <interfaces/IPhysicsEngine.hpp>
#include <types/types.hpp>

namespace physics {
    class RelativityPhysics : public common::IPhysicsEngine {
        public:
            RelativityPhysics() = default;
            ~RelativityPhysics() override = default;

            void init(common::SpecificDataPhysics world) override;

            void update(double dt) override;

            [[nodiscard]] double stepDuration() const override { return STEP_DURATION; }

            void physicsPause() override { this->_state = common::ModuleState::PAUSE; };

            void physicsResume() override { this->_state = common::ModuleState::RUNNING; };

            void shutdown() override;

            void syncIn(common::SpecificDataPhysics world) override;
            common::WorldState publish() override;

            [[nodiscard]] std::string getName() const override { return "RelativityPhysics"; }

        private:
            static constexpr double STEP_DURATION = 7200.0;

            void _publishOrientations(common::WorldState& world, std::size_t count) const;

            common::SpecificDataPhysics _worldState;
            common::ModuleState _state;
    };
} // namespace physics
