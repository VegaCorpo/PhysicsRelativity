#pragma once

#include <expected>
#include <integration/TimeStep.hpp>
#include <interfaces/IPhysicsEngine.hpp>
#include <memory>
#include <string>
#include <vector>
#include "components/RelativisticState.hpp"
#include "integration/Bodies.hpp"
#include "types/types.hpp"

namespace physics {
    class RelativityPhysics : public common::IPhysicsEngine {
        public:
            RelativityPhysics(std::unique_ptr<integration::IAccelerationField> field,
                              std::unique_ptr<integration::IIntegrator> integrator);
            ~RelativityPhysics() override = default;

            void init(common::SpecificDataPhysics world) override;

            void update(double dt) override;

            [[nodiscard]] double stepDuration() const override { return this->_timeStep.maxSubStep(); }

            void physicsPause() override { this->_state = common::ModuleState::PAUSE; };

            void physicsResume() override { this->_state = common::ModuleState::RUNNING; };

            void shutdown() override;

            void syncIn(common::SpecificDataPhysics world) override;
            common::WorldState publish() override;

            [[nodiscard]] std::string getName() const override { return "RelativityPhysics"; }

            [[nodiscard]] const std::vector<double>& lorentzFactors() const noexcept
            {
                return this->_relativisticState.lorentzFactors;
            }

            [[nodiscard]] const std::vector<double>& properTimes() const noexcept
            {
                return this->_relativisticState.properTimes;
            }

        private:
            [[nodiscard]] static common::integration::TimeStep _timeStepOf(const integration::IIntegrator& integrator);
            [[nodiscard]] bool _canAdvance(
                const std::expected<common::integration::SubSteps, common::integration::TimeStepError>& subSteps) const;
            void _advance(const common::integration::SubSteps& subSteps);
            void _advanceOneStep(double duration);
            [[nodiscard]] std::size_t _publishedCount() const;

            std::unique_ptr<integration::IAccelerationField> _field;
            std::unique_ptr<integration::IIntegrator> _integrator;
            common::integration::TimeStep _timeStep;
            common::SpecificDataPhysics _worldState;
            RelativisticState _relativisticState;
            integration::IntegrationWorkspace _workspace;
            common::ModuleState _state = common::ModuleState::RUNNING;
    };
} // namespace physics
