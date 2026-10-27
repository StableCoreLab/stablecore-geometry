#include <limits>
#include <numbers>

#include <gtest/gtest.h>

#include "Core/PathGeometry.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"

using namespace Geometry;

namespace
{
    const SCPathGeometryTolerance kTolerance{1e-8, 1e-8, 1e-12};
}

TEST(PathGeometryTest, CornerReturnsInnerAngleAndSignedTurn)
{
    const SCLineSegment2d incoming({0,0}, {1,0});
    const SCLineSegment2d left({1,0}, {1,1});
    const SCLineSegment2d reverse({1,0}, {0,0});
    const auto corner = EvaluateCornerGeometry(incoming, left, kTolerance);
    ASSERT_TRUE(corner.success);
    EXPECT_EQ(corner.turn, SCCornerTurn2d::Left);
    EXPECT_NEAR(corner.interiorAngleRad, std::numbers::pi_v<double> / 2.0, 1e-12);
    const auto reversal = EvaluateCornerGeometry(incoming, reverse, kTolerance);
    ASSERT_TRUE(reversal.success);
    EXPECT_EQ(reversal.turn, SCCornerTurn2d::Reversal);
    EXPECT_DOUBLE_EQ(reversal.signedTurnRad, std::numbers::pi_v<double>);
    const SCLineSegment2d right({1,0}, {1,-1});
    const auto rightCorner = EvaluateCornerGeometry(incoming, right, kTolerance);
    ASSERT_TRUE(rightCorner.success);
    EXPECT_EQ(rightCorner.turn, SCCornerTurn2d::Right);
    EXPECT_NEAR(rightCorner.signedTurnRad, -std::numbers::pi_v<double> / 2.0, 1e-12);
    const SCLineSegment2d straight({1,0}, {2,0});
    const auto straightCorner = EvaluateCornerGeometry(incoming, straight, kTolerance);
    ASSERT_TRUE(straightCorner.success);
    EXPECT_EQ(straightCorner.turn, SCCornerTurn2d::Straight);
    EXPECT_DOUBLE_EQ(straightCorner.interiorAngleRad, std::numbers::pi_v<double>);
}

TEST(PathGeometryTest, ArcCornerUsesEndpointTangents)
{
    const SCLineSegment2d incoming({-1,0}, {0,0});
    const SCArcSegment2d outgoing({0,1}, 1.0, -std::numbers::pi_v<double> / 2.0,
                                  std::numbers::pi_v<double> / 2.0);
    const auto corner = EvaluateCornerGeometry(incoming, outgoing, kTolerance);
    ASSERT_TRUE(corner.success);
    EXPECT_EQ(corner.turn, SCCornerTurn2d::Straight);
    EXPECT_DOUBLE_EQ(corner.interiorAngleRad, std::numbers::pi_v<double>);
}

TEST(PathGeometryTest, CornerNormalizesToleranceBandsAndArcArcTangents)
{
    const double pi = std::numbers::pi_v<double>;
    const SCLineSegment2d incoming({0,0}, {1,0});
    const SCLineSegment2d nearStraight({1,0}, {2,5e-9});
    const auto straight = EvaluateCornerGeometry(incoming, nearStraight, kTolerance);
    ASSERT_TRUE(straight.success);
    EXPECT_EQ(straight.turn, SCCornerTurn2d::Straight);
    EXPECT_DOUBLE_EQ(straight.signedTurnRad, 0.0);

    const SCLineSegment2d beyondStraight({1,0}, {2,2e-8});
    const auto left = EvaluateCornerGeometry(incoming, beyondStraight, kTolerance);
    ASSERT_TRUE(left.success);
    EXPECT_EQ(left.turn, SCCornerTurn2d::Left);
    EXPECT_GT(left.signedTurnRad, kTolerance.angleToleranceRad);

    const SCLineSegment2d nearNegativeReversal({1,0}, {0,-5e-9});
    const auto reversal = EvaluateCornerGeometry(incoming, nearNegativeReversal, kTolerance);
    ASSERT_TRUE(reversal.success);
    EXPECT_EQ(reversal.turn, SCCornerTurn2d::Reversal);
    EXPECT_DOUBLE_EQ(reversal.signedTurnRad, pi);
    EXPECT_DOUBLE_EQ(reversal.interiorAngleRad, 0.0);

    const SCArcSegment2d arcIn({0,1}, 1.0, pi, pi / 2.0);
    const SCArcSegment2d arcOut({0,-1}, 1.0, pi / 2.0, -pi / 2.0);
    const auto arcCorner = EvaluateCornerGeometry(arcIn, arcOut, kTolerance);
    ASSERT_TRUE(arcCorner.success);
    EXPECT_EQ(arcCorner.turn, SCCornerTurn2d::Straight);
    EXPECT_DOUBLE_EQ(arcCorner.interiorAngleRad, pi);
}

TEST(PathGeometryTest, CornerRejectsGapZeroDirectionAndNonFiniteInput)
{
    const SCLineSegment2d incoming({0,0}, {1,0});
    const SCLineSegment2d gapped({2,0}, {3,0});
    const auto gap = EvaluateCornerGeometry(incoming, gapped, kTolerance);
    EXPECT_EQ(gap.failure, SCPathGeometryFailure::DiscontinuousPath);
    EXPECT_EQ(gap.inputIndex, 0u);
    EXPECT_EQ(gap.relatedIndex, 1u);

    const SCLineSegment2d zero({1,0}, {1,0});
    const auto degenerate = EvaluateCornerGeometry(incoming, zero, kTolerance);
    EXPECT_EQ(degenerate.failure, SCPathGeometryFailure::DegenerateDirection);
    EXPECT_EQ(degenerate.inputIndex, 1u);

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const SCLineSegment2d invalid({1,0}, {nan,0});
    const auto rejected = EvaluateCornerGeometry(incoming, invalid, kTolerance);
    EXPECT_EQ(rejected.failure, SCPathGeometryFailure::NonFiniteInput);
    EXPECT_EQ(rejected.inputIndex, 1u);
}
