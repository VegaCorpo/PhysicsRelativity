#pragma once

#include <interfaces/IPhysicsEngine.hpp>

namespace physics {
    class RelativityPhysics : public common::IPhysicsEngine {
        public:
            RelativityPhysics() = default;
            ~RelativityPhysics() override = default;

            void init(common::WorldState world) override;

            void update(double dt) override;

            void shutdown() override;

            void syncIn(common::WorldState world) override;
            common::WorldState syncOut() override;

            [[nodiscard]] std::string getName() const override { return "RelativityPhysics"; }

        private:
            common::WorldState _worldState;
    };
} // namespace physics
