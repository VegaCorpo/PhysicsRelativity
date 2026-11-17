#pragma once

#include <array>
#include <cstddef>
#include "integration/Bodies.hpp"

namespace physics::integration {
    class Rk4 : public IIntegrator {
        public:
            static constexpr std::size_t FORCE_EVALUATIONS_PER_STEP = 4;

            void step(const Bodies& bodies, IAccelerationField& field, double duration) const override;

            [[nodiscard]] std::size_t forceEvaluationsPerStep() const noexcept override
            {
                return FORCE_EVALUATIONS_PER_STEP;
            }

        private:
            struct Stage {
                    double weight;
                    double nextOffset;
            };

            static constexpr std::array<Stage, 3> INTERMEDIATE_STAGES{{{1.0, 0.5}, {2.0, 0.5}, {2.0, 1.0}}};
            static constexpr double FINAL_STAGE_WEIGHT = 1.0;
            static constexpr double STAGE_WEIGHT_SUM = 6.0;

            static void _saveInitialState(const Bodies& bodies);
            static void _evaluateStage(const Bodies& bodies, IAccelerationField& field, double weight);
            static void _moveToStage(const Bodies& bodies, double offset);
            static void _combineStages(const Bodies& bodies, double duration);
    };
} // namespace physics::integration
