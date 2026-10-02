#pragma once

#include <interfaces/IPhysicsEngine.hpp>

namespace physics {
    class RelativityPhysics : public common::IPhysicsEngine {
        public:
            RelativityPhysics() = default;
            ~RelativityPhysics() override = default;

            void init(common::SpecificDataPhysics world) override;

            void update(double dt) override;

            void shutdown() override;

            void syncIn(common::SpecificDataPhysics world) override;
            common::WorldState publish() override;

            [[nodiscard]] std::string getName() const override { return "RelativityPhysics"; }

        private:
            void _publishOrientations(common::WorldState& world, std::size_t count) const;

            common::SpecificDataPhysics _worldState;
    };
} // namespace physics
