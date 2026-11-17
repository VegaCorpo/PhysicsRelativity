#include "utils/RelativityUtils.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <utils/physicalConstants.hpp>

namespace {
    using namespace physics::utils;
    constexpr double SPEED_OF_LIGHT = common::constants::SPEED_OF_LIGHT;
    constexpr RelativeVelocity vx{0.5 * SPEED_OF_LIGHT, 0.0, 0.0};
    constexpr SpacetimeEvent event{2.0, 1000.0, 200.0, -50.0};
    double interval(const SpacetimeEvent& e)
    {
        return minkowskiIntervalSquared(e.time, e.x, e.y, e.z);
    }
} // namespace

TEST(Relativity, LorentzFactorRejectsNonSubluminalAndNonFiniteSpeeds)
{
    EXPECT_DOUBLE_EQ(*lorentzFactor({0, 0, 0}), 1.0);
    EXPECT_NEAR(*lorentzFactor(vx), 1.0 / std::sqrt(0.75), 1e-15);
    EXPECT_EQ(lorentzFactor({SPEED_OF_LIGHT, 0, 0}).error(), LorentzTransformError::SPEED_NOT_SUBLUMINAL);
    EXPECT_EQ(lorentzFactor({std::numeric_limits<double>::infinity(), 0, 0}).error(),
              LorentzTransformError::NON_FINITE_INPUT);
}

TEST(Relativity, BoostsAlongEachAxisPreserveMinkowskiInterval)
{
    for (const RelativeVelocity boost : {RelativeVelocity{0.4 * SPEED_OF_LIGHT, 0, 0},
                                         {0, -0.4 * SPEED_OF_LIGHT, 0},
                                         {0, 0, 0.4 * SPEED_OF_LIGHT},
                                         {0.2 * SPEED_OF_LIGHT, 0.2 * SPEED_OF_LIGHT, 0.2 * SPEED_OF_LIGHT}}) {
        auto transformed = transformToMovingFrame(event, boost);
        ASSERT_TRUE(transformed);
        EXPECT_NEAR(interval(*transformed), interval(event), std::abs(interval(event)) * 1e-12 + 1e-5);
    }
}

TEST(Relativity, InverseBoostReturnsOriginalEvent)
{
    const RelativeVelocity boost{0.2 * SPEED_OF_LIGHT, -0.1 * SPEED_OF_LIGHT, 0.05 * SPEED_OF_LIGHT};
    auto transformed = transformToMovingFrame(event, boost);
    ASSERT_TRUE(transformed);
    auto restored = transformFromMovingFrame(*transformed, boost);
    ASSERT_TRUE(restored);
    EXPECT_NEAR(restored->time, event.time, 1e-12);
    EXPECT_NEAR(restored->x, event.x, 1e-10);
    EXPECT_NEAR(restored->y, event.y, 1e-10);
    EXPECT_NEAR(restored->z, event.z, 1e-10);
}

TEST(Relativity, SpatialRotationPreservesIntervalAndTime)
{
    auto rotated = rotateSpatially(event, 1.2, {0, 0, 1});
    ASSERT_TRUE(rotated);
    EXPECT_DOUBLE_EQ(rotated->time, event.time);
    EXPECT_NEAR(interval(*rotated), interval(event), std::abs(interval(event)) * 1e-12 + 1e-5);
}

TEST(Relativity, VelocityCompositionStaysSubluminalAndHandlesCollinearCase)
{
    auto composed = transformVelocityToMovingFrame({0.5 * SPEED_OF_LIGHT, 0, 0}, {0.5 * SPEED_OF_LIGHT, 0, 0});
    ASSERT_TRUE(composed);
    EXPECT_NEAR(composed->x, 0.0, 1e-10);
    EXPECT_LT(speedMagnitude(*composed), SPEED_OF_LIGHT);
    EXPECT_EQ(transformVelocityToMovingFrame({SPEED_OF_LIGHT, 0, 0}, vx).error(),
              LorentzTransformError::SPEED_NOT_SUBLUMINAL);
}

TEST(Relativity, InvalidEventAndFrameAreRejected)
{
    EXPECT_EQ(transformToMovingFrame({std::numeric_limits<double>::quiet_NaN(), 0, 0, 0}, vx).error(),
              LorentzTransformError::NON_FINITE_INPUT);
    EXPECT_EQ(transformToMovingFrame(event, {SPEED_OF_LIGHT, 0, 0}).error(),
              LorentzTransformError::SPEED_NOT_SUBLUMINAL);
}
