#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

#include "Core/ArcConstruction2d.h"
#include "Core/Transform.h"
#include "Core/Projection.h"
#include "Types/Geometry2d/SCTransform2d.h"

using namespace Geometry;

TEST(ArcConstruction2dTest, ThreePointsSelectDirectionAndPassThroughMiddle)
{
    for (double sign : {1.0, -1.0})
    {
        SCArcSegment2d arc;
        ASSERT_TRUE(CreateArcThroughThreePoints2d({1, 0}, {0, sign}, {-1, 0}, arc));
        EXPECT_TRUE(arc.center.AlmostEquals(SCPoint2d{0, 0}));
        EXPECT_DOUBLE_EQ(arc.radius, 1.0);
        EXPECT_NEAR(arc.sweepAngle, sign * std::numbers::pi_v<double>, 1e-12);
        EXPECT_TRUE(arc.PointAt(0.5).AlmostEquals(SCPoint2d{0, sign}));
    }
    // through 决定大弧，不能仅按起终点选择小弧。
    const auto major = SolveArcThroughThreePoints({1, 0}, {-1, 0}, {0, 1});
    ASSERT_TRUE(major);
    EXPECT_NEAR(major->arc.sweepAngle, -1.5 * std::numbers::pi_v<double>, 1e-12);
}

TEST(ArcConstruction2dTest, ThreePointsAreStableUnderTranslationAndScaling)
{
    for (double scale : {1e-6, 1.0, 1e150})
    {
        const SCPoint2d center{3 * scale, -2 * scale};
        const auto result = SolveArcThroughThreePoints(
            {center.x + scale, center.y}, {center.x, center.y + scale},
            {center.x - scale, center.y}, scale * 1e-9);
        ASSERT_TRUE(result);
        EXPECT_NEAR(result->arc.radius / scale, 1.0, 1e-12);
        EXPECT_NEAR(result->arc.center.x / scale, 3.0, 1e-12);
        EXPECT_NEAR(result->arc.center.y / scale, -2.0, 1e-12);
        EXPECT_LE((result->arc.StartPoint() - SCPoint2d{center.x + scale, center.y}).Length(), scale * 1e-9);
        EXPECT_LE((result->arc.EndPoint() - SCPoint2d{center.x - scale, center.y}).Length(), scale * 1e-9);
        EXPECT_LE((ProjectPointToArcSegment({center.x, center.y + scale}, result->arc).point -
                   SCPoint2d{center.x, center.y + scale}).Length(), scale * 1e-9);
    }
    const auto translated = SolveArcThroughThreePoints(
        {1e12 + 1, 1e12}, {1e12, 1e12 + 1}, {1e12 - 1, 1e12});
    ASSERT_TRUE(translated);
    EXPECT_DOUBLE_EQ(translated->arc.radius, 1.0);
}

TEST(ArcConstruction2dTest, RadiusSupportsAllFourSelections)
{
    for (auto direction : {SCArcDirection2d::CounterClockwise, SCArcDirection2d::Clockwise})
    {
        for (bool large : {false, true})
        {
            SCArcSegment2d arc;
            ASSERT_TRUE(CreateArcFromStartEndRadius2d({1, 0}, {0, 1}, 1, direction, large, arc));
            EXPECT_EQ(arc.Direction(), direction);
            EXPECT_EQ(std::abs(arc.sweepAngle) > std::numbers::pi_v<double>, large);
            EXPECT_TRUE(arc.StartPoint().AlmostEquals(SCPoint2d{1, 0}));
            EXPECT_TRUE(arc.EndPoint().AlmostEquals(SCPoint2d{0, 1}));
        }
    }
}

TEST(ArcConstruction2dTest, ClampingAndSemicircleClassificationAreExplicit)
{
    for (auto direction : {SCArcDirection2d::CounterClockwise, SCArcDirection2d::Clockwise})
    {
        for (double radius : {0.5, 1.0})
        {
            const auto result = SolveArcFromStartEndRadius({0, 0}, {2, 0}, radius, direction, false);
            ASSERT_TRUE(result);
            EXPECT_EQ(result->radiusClamped, radius < 1.0);
            EXPECT_DOUBLE_EQ(result->arc.radius, 1.0);
            EXPECT_DOUBLE_EQ(std::abs(result->arc.sweepAngle), std::numbers::pi_v<double>);
            EXPECT_FALSE(SolveArcFromStartEndRadius({0, 0}, {2, 0}, radius, direction, true));
        }
    }
    // 临时实现的 r*r 在这个尺度溢出；正式实现应仍能构造。
    EXPECT_TRUE(SolveArcFromStartEndRadius({0, 0}, {1e160, 0}, 1e160,
                                         SCArcDirection2d::Clockwise, false, 1e151));
}

TEST(ArcConstruction2dTest, RoundedCentersRejectInaccurateArcsWithoutModifyingOutput)
{
    const SCPoint2d start{1e16, 0};
    const SCPoint2d end{1e16 + 2, 0};
    const SCPoint2d through{1e16 + 2, 2};
    const SCArcSegment2d original({3, 4}, 2, 0, 1);
    SCArcSegment2d out = original;
    EXPECT_FALSE(SolveArcThroughThreePoints(start, through, end));
    EXPECT_FALSE(CreateArcThroughThreePoints2d(start, through, end, out));
    EXPECT_TRUE(out.AlmostEquals(original));
    for (auto direction : {SCArcDirection2d::CounterClockwise, SCArcDirection2d::Clockwise})
    {
        for (bool large : {false, true})
        {
            EXPECT_FALSE(SolveArcFromStartEndRadius(start, end, 2, direction, large));
            EXPECT_FALSE(CreateArcFromStartEndRadius2d(start, end, 2, direction, large, out));
            EXPECT_TRUE(out.AlmostEquals(original));
        }
    }
    // 大坐标本身不构成失败：可表示且满足输入点约束的圆弧仍应交付。
    const auto exact = SolveArcThroughThreePoints({1e16 + 2, 0}, {1e16, 2}, {1e16 - 2, 0});
    ASSERT_TRUE(exact);
    EXPECT_LE((exact->arc.StartPoint() - SCPoint2d{1e16 + 2, 0}).Length(), kDefaultEpsilon);
    EXPECT_LE((exact->arc.EndPoint() - SCPoint2d{1e16 - 2, 0}).Length(), kDefaultEpsilon);
}

TEST(ArcConstruction2dTest, EndpointResidualUsesExplicitDistanceTolerance)
{
    // 无平移的有限圆弧也可能因大半径下的角度舍入而偏离端点。
    EXPECT_FALSE(SolveArcFromStartEndRadius({0, 0}, {1e160, 0}, 1e160,
                                          SCArcDirection2d::Clockwise, false));
    constexpr double eps = 1e151;
    const auto relaxed = SolveArcFromStartEndRadius({0, 0}, {1e160, 0}, 1e160,
                                                   SCArcDirection2d::Clockwise, false, eps);
    ASSERT_TRUE(relaxed);
    EXPECT_LE((relaxed->arc.StartPoint() - SCPoint2d{0, 0}).Length(), eps);
    EXPECT_LE((relaxed->arc.EndPoint() - SCPoint2d{1e160, 0}).Length(), eps);
}

TEST(ArcConstruction2dTest, InvalidInputsDoNotModifyOutput)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    SCArcSegment2d arc({3, 4}, 2, 0, 1);
    const SCArcSegment2d original = arc;
    EXPECT_FALSE(CreateArcThroughThreePoints2d({0, 0}, {1, 1}, {2, 2}, arc));
    EXPECT_FALSE(CreateArcThroughThreePoints2d({0, 0}, {0, 0}, {2, 0}, arc));
    EXPECT_FALSE(CreateArcThroughThreePoints2d({nan, 0}, {0, 1}, {2, 0}, arc));
    for (double radius : {0.0, -1.0, nan, inf})
        EXPECT_FALSE(CreateArcFromStartEndRadius2d({0, 0}, {2, 0}, radius,
                                                 SCArcDirection2d::Clockwise, false, arc));
    EXPECT_FALSE(CreateArcFromStartEndRadius2d({0, 0}, {0, 0}, 1,
                                             SCArcDirection2d::Clockwise, false, arc));
    EXPECT_FALSE(CreateArcFromStartEndRadius2d({0, 0}, {1, 0}, 1,
                                             static_cast<SCArcDirection2d>(99), false, arc));
    EXPECT_TRUE(arc.AlmostEquals(original));
    for (double eps : {0.0, -1.0, nan, inf})
    {
        EXPECT_FALSE(SolveArcThroughThreePoints({1, 0}, {0, 1}, {-1, 0}, eps));
        EXPECT_FALSE(SolveArcFromStartEndRadius({0, 0}, {1, 0}, 1,
                                              SCArcDirection2d::Clockwise, false, eps));
    }
}

TEST(Reflection2dTest, AxesDiagonalAndTranslatedAxis)
{
    const SCPoint2d point{3, 4};
    const SCPoint2d origins[] = {{0, 0}, {0, 0}, {0, 0}, {5, 0}};
    const SCVector2d directions[] = {{1, 0}, {0, 2}, {1, 1}, {0, 1}};
    const SCPoint2d expected[] = {{3, -4}, {-3, 4}, {4, 3}, {7, 4}};
    for (int i = 0; i < 4; ++i)
    {
        const auto reflection = SCTransform2d::Reflection(origins[i], directions[i]);
        ASSERT_TRUE(reflection);
        EXPECT_TRUE(reflection->Apply(point).AlmostEquals(expected[i]));
        EXPECT_TRUE(reflection->Apply(origins[i]).AlmostEquals(origins[i]));
        EXPECT_TRUE(reflection->Apply(reflection->Apply(point)).AlmostEquals(point));
        EXPECT_NEAR(reflection->linear.Determinant(), -1.0, 1e-12);
        ASSERT_TRUE(reflection->Inverse());
        EXPECT_TRUE(reflection->Inverse()->Apply(point).AlmostEquals(expected[i]));
        EXPECT_TRUE(Mirror(point, origins[i], directions[i]).AlmostEquals(expected[i]));
        const SCArcSegment2d arc({1, 2}, 3, 0.25, 1.5);
        const auto mirrored = Mirror(arc, origins[i], directions[i]);
        EXPECT_TRUE(mirrored.StartPoint().AlmostEquals(reflection->Apply(arc.StartPoint())));
        EXPECT_TRUE(mirrored.EndPoint().AlmostEquals(reflection->Apply(arc.EndPoint())));
        EXPECT_DOUBLE_EQ(mirrored.sweepAngle, -arc.sweepAngle);
    }
}

TEST(Reflection2dTest, InvalidAxisReturnsNullopt)
{
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(SCTransform2d::Reflection({0, 0}, {0, 0}));
    EXPECT_FALSE(SCTransform2d::Reflection({inf, 0}, {1, 0}));
    EXPECT_FALSE(SCTransform2d::Reflection({0, 0}, {inf, 1}));
    EXPECT_FALSE(SCTransform2d::Reflection({0, 0}, {1, 0}, -1));
    EXPECT_TRUE(Mirror(SCPoint2d{3, 4}, {0, 0}, {0, 0}).AlmostEquals(SCPoint2d{3, 4}));
}
