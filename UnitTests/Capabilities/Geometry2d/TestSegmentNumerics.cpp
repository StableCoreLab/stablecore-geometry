#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include "Core/Intersection.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Support/Epsilon.h"

using Geometry::SCArcSegment2d;
using Geometry::SCLineSegment2d;

TEST(SegmentTest, ArcComparisonSeparatesDistanceAndAngleTolerance)
{
    const SCArcSegment2d arc({0.0, 0.0}, 2.0, 0.25, 1.0);
    const SCArcSegment2d rotated({0.0, 0.0}, 2.0, 0.2501, 1.0);
    const SCArcSegment2d moved({2e-6, 0.0}, 2.0, 0.25, 1.0);

    EXPECT_TRUE(arc.AlmostEquals(rotated, 1e-6, 1e-3));
    EXPECT_FALSE(arc.AlmostEquals(rotated, 1e-6, 1e-5));
    EXPECT_FALSE(arc.AlmostEquals(moved, 1e-6, 1e-3));
    EXPECT_FALSE(arc.AlmostEquals(arc, -1.0, 1e-3));
    EXPECT_FALSE(arc.AlmostEquals(arc, 1e-3, std::numeric_limits<double>::infinity()));
    EXPECT_TRUE(arc.AlmostEquals(rotated, 1e-3));
}

TEST(SegmentTest, LineIntersectionHandlesFiniteLargeDirections)
{
    const Geometry::SCLine2d horizontal = Geometry::SCLine2d::FromOriginAndDirection(
        {-1.0, 0.0}, {1e200, 0.0});
    const Geometry::SCLine2d vertical = Geometry::SCLine2d::FromOriginAndDirection(
        {0.0, -1.0}, {0.0, 1e200});
    ASSERT_TRUE(horizontal.IsValid());
    ASSERT_TRUE(vertical.IsValid());

    const auto result = Geometry::Intersect(horizontal, vertical);
    ASSERT_EQ(result.pointCount, 1U);
    EXPECT_EQ(result.kind, Geometry::SCIntersectionKind2d::Point);
    EXPECT_TRUE(result.points[0].point.AlmostEquals({0.0, 0.0}, 1e-12));
    EXPECT_NEAR(result.points[0].parameterOnFirst, 1e-200, 1e-212);
    EXPECT_NEAR(result.points[0].parameterOnSecond, 1e-200, 1e-212);
}

TEST(SegmentTest, LineArcIntersectionHandlesFiniteLargeDirection)
{
    const SCLineSegment2d line({0.0, 0.0}, {1e200, 0.0});
    const SCArcSegment2d circle({2.0, 0.0}, 1.0, 0.0, Geometry::kTwoPi);
    ASSERT_TRUE(line.IsValid());
    ASSERT_TRUE(circle.IsValid());

    const auto direct = Geometry::Intersect(line, circle);
    ASSERT_EQ(direct.pointCount, 2U);
    EXPECT_NEAR(direct.points[0].point.x, 1.0, 1e-12);
    EXPECT_NEAR(direct.points[1].point.x, 3.0, 1e-12);

    const auto extended = Geometry::IntersectExtended(line, circle, Geometry::SCExtensionPolicy::ExtendFirst);
    ASSERT_EQ(extended.pointCount, 2U);
    EXPECT_NEAR(extended.points[0].point.x, 1.0, 1e-12);
    EXPECT_NEAR(extended.points[1].point.x, 3.0, 1e-12);
}

TEST(SegmentTest, ExtendedLineArcIntersectionHandlesOverflowingCenterProjection)
{
    const SCLineSegment2d line({0.0, 0.0}, {1e308, 1e308});
    const SCArcSegment2d circle({1.5e308, 1.5e308}, 1e300, 0.0, Geometry::kTwoPi);
    ASSERT_TRUE(line.IsValid());
    ASSERT_TRUE(circle.IsValid());

    const auto result = Geometry::IntersectExtended(line, circle, Geometry::SCExtensionPolicy::ExtendFirst);
    ASSERT_EQ(result.pointCount, 2U);
    EXPECT_TRUE(result.points[0].point.IsValid());
    EXPECT_TRUE(result.points[1].point.IsValid());
    EXPECT_NEAR(result.points[0].parameterOnFirst, 1.5, 1e-7);
    EXPECT_NEAR(result.points[1].parameterOnFirst, 1.5, 1e-7);
    for (const auto& hit : result.points)
    {
        EXPECT_NEAR(std::hypot(hit.point.x - circle.center.x, hit.point.y - circle.center.y),
                    circle.radius, 1e293);
    }
    EXPECT_FALSE(result.onFirstSegment);
    EXPECT_TRUE(result.onSecondSegment);
}

TEST(SegmentTest, LineValidityRejectsOverflowingDirectionLength)
{
    const Geometry::SCLine2d line = Geometry::SCLine2d::FromOriginAndDirection(
        {0.0, 0.0}, {1.5e308, 1.5e308});
    ASSERT_TRUE(line.direction.IsValid());
    EXPECT_FALSE(line.IsValid());
}

TEST(SegmentTest, LineIntersectionRetainsRepresentableTinySlope)
{
    const Geometry::SCLine2d horizontal = Geometry::SCLine2d::FromOriginAndDirection(
        {0.0, 0.0}, {1e308, 0.0});
    const Geometry::SCLine2d sloped = Geometry::SCLine2d::FromOriginAndDirection(
        {0.0, 1e-320}, {1e308, 1e-20});

    const auto result = Geometry::Intersect(horizontal, sloped, 0.0);
    ASSERT_EQ(result.pointCount, 1U);
    EXPECT_EQ(result.kind, Geometry::SCIntersectionKind2d::Point);
    const double expectedX = -(sloped.origin.y * horizontal.direction.x) / sloped.direction.y;
    EXPECT_NEAR(result.points[0].point.x, expectedX, 1e-3);
    EXPECT_NEAR(result.points[0].point.y, 0.0, 1e-320);
}

TEST(SegmentTest, LineIntersectionHandlesUnderflowingRawCrossProduct)
{
    const Geometry::SCLine2d horizontal = Geometry::SCLine2d::FromOriginAndDirection(
        {0.0, 0.0}, {1e-200, 0.0});
    const Geometry::SCLine2d vertical = Geometry::SCLine2d::FromOriginAndDirection(
        {0.0, 0.0}, {0.0, 1e-200});

    const auto result = Geometry::Intersect(horizontal, vertical, 0.0);
    ASSERT_EQ(result.pointCount, 1U);
    EXPECT_EQ(result.kind, Geometry::SCIntersectionKind2d::Point);
    EXPECT_TRUE(result.points[0].point.AlmostEquals({0.0, 0.0}, 0.0));
}

TEST(SegmentTest, LineIntersectionHandlesOverflowingOriginDifference)
{
    const Geometry::SCLine2d vertical = Geometry::SCLine2d::FromOriginAndDirection(
        {1e308, 0.0}, {0.0, 1.0});
    const Geometry::SCLine2d horizontal = Geometry::SCLine2d::FromOriginAndDirection(
        {-1e308, 0.0}, {1e308, 0.0});
    ASSERT_TRUE(vertical.IsValid());
    ASSERT_TRUE(horizontal.IsValid());

    const auto result = Geometry::Intersect(vertical, horizontal);
    ASSERT_EQ(result.pointCount, 1U);
    EXPECT_EQ(result.kind, Geometry::SCIntersectionKind2d::Point);
    EXPECT_EQ(result.points[0].point.x, 1e308);
    EXPECT_EQ(result.points[0].point.y, 0.0);
    EXPECT_EQ(result.points[0].parameterOnFirst, 0.0);
    EXPECT_EQ(result.points[0].parameterOnSecond, 2.0);
}

TEST(SegmentTest, ExplicitToleranceLineArcIntersectionPreservesSmallSweepParameter)
{
    constexpr double eps = 1e-12;
    const SCArcSegment2d arc({0.0, 0.0}, 1.0, 0.0, 1e-10);
    const SCLineSegment2d line({0.9, 5e-11}, {1.1, 5e-11});
    ASSERT_FALSE(arc.IsValid());
    ASSERT_TRUE(arc.IsValid(eps));
    const auto result = Geometry::Intersect(line, arc, eps);
    ASSERT_EQ(result.pointCount, 1U);
    EXPECT_NEAR(result.points[0].point.y, 5e-11, eps);
    EXPECT_NEAR(result.points[0].parameterOnSecond, 0.5, 1e-5);
}

TEST(SegmentTest, ExplicitToleranceLineArcIntersectionUsesLinearSeparation)
{
    constexpr double eps = 1e-12;
    const SCArcSegment2d arc({0.0, 0.0}, 1e-8, 0.0, Geometry::kPi);
    const SCLineSegment2d crossing({-2e-8, 0.0}, {2e-8, 0.0});
    const auto twoPoints = Geometry::Intersect(crossing, arc, eps);
    ASSERT_EQ(twoPoints.pointCount, 2U);
    EXPECT_EQ(twoPoints.kind, Geometry::SCIntersectionKind2d::Point);

    const SCLineSegment2d separated({-2e-8, 1.1e-8}, {2e-8, 1.1e-8});
    const auto noPoint = Geometry::Intersect(separated, arc, eps);
    EXPECT_EQ(noPoint.pointCount, 0U);
}

TEST(SegmentTest, ExplicitToleranceArcArcIntersectionAcceptsSmallSweep)
{
    constexpr double eps = 1e-12;
    const SCArcSegment2d first({0.0, 0.0}, 1.0, Geometry::kPi / 3.0 - 5e-11, 1e-10);
    const SCArcSegment2d second({1.0, 0.0}, 1.0, 0.0, Geometry::kTwoPi);
    ASSERT_FALSE(first.IsValid());
    ASSERT_TRUE(first.IsValid(eps));
    const auto result = Geometry::Intersect(first, second, eps);
    ASSERT_EQ(result.pointCount, 1U);
    EXPECT_EQ(result.kind, Geometry::SCIntersectionKind2d::Point);
    EXPECT_NEAR(result.points[0].point.x, 0.5, eps);
    EXPECT_NEAR(result.points[0].parameterOnFirst, 0.5, 1e-5);
}

TEST(SegmentTest, ExplicitToleranceArcArcIntersectionUsesAngleTolerance)
{
    constexpr double eps = 1e-12;
    const SCArcSegment2d first({0.0, 0.0}, 1e-8, 0.0, Geometry::kPi / 3.0 - 1e-6);
    const SCArcSegment2d second({1e-8, 0.0}, 1e-8, 0.0, Geometry::kTwoPi);
    const auto result = Geometry::Intersect(first, second, eps);
    ASSERT_EQ(result.pointCount, 1U);
    EXPECT_NEAR(result.points[0].point.x, 5e-9, eps);
    EXPECT_NEAR(result.points[0].point.y, std::sqrt(3.0) * 5e-9, eps);
}

TEST(SegmentTest, ExplicitToleranceExtendedLineArcPreservesSmallSweep)
{
    constexpr double eps = 1e-12;
    const SCArcSegment2d arc({0.0, 0.0}, 1.0, 0.0, 1e-10);
    const SCLineSegment2d line({0.9, 5e-11}, {1.1, 5e-11});
    const auto result = Geometry::IntersectExtended(line, arc, Geometry::SCExtensionPolicy::ExtendFirst, eps);
    ASSERT_EQ(result.pointCount, 1U);
    EXPECT_TRUE(result.onFirstSegment);
    EXPECT_TRUE(result.onSecondSegment);
    EXPECT_NEAR(result.points[0].parameterOnSecond, 0.5, 1e-5);
}

TEST(SegmentTest, ExplicitToleranceExtendedArcArcPreservesSmallSweep)
{
    constexpr double eps = 1e-12;
    const SCArcSegment2d first({0.0, 0.0}, 1.0, Geometry::kPi / 3.0 - 5e-11, 1e-10);
    const SCArcSegment2d second({1.0, 0.0}, 1.0, 0.0, Geometry::kTwoPi);
    const auto result = Geometry::IntersectExtended(first, second, Geometry::SCExtensionPolicy::ExtendSecond, eps);
    ASSERT_EQ(result.pointCount, 1U);
    EXPECT_TRUE(result.onFirstSegment);
    EXPECT_TRUE(result.onSecondSegment);
    EXPECT_NEAR(result.points[0].parameterOnFirst, 0.5, 1e-5);
}

TEST(SegmentTest, ExplicitToleranceExtendedShortLinesRemainTransverse)
{
    constexpr double eps = 1e-12;
    const SCLineSegment2d first({-5e-11, 0.0}, {5e-11, 0.0});
    const SCLineSegment2d second({0.0, -5e-11}, {0.0, 5e-11});
    const auto result = Geometry::IntersectExtended(first, second, Geometry::SCExtensionPolicy::ExtendBoth, eps);
    ASSERT_EQ(result.pointCount, 1U);
    EXPECT_EQ(result.kind, Geometry::SCIntersectionKind2d::Point);
    EXPECT_TRUE(result.onFirstSegment);
    EXPECT_TRUE(result.onSecondSegment);
}

TEST(SegmentTest, ExplicitToleranceExtendedLineArcAcceptsShortLine)
{
    constexpr double eps = 1e-12;
    const SCLineSegment2d line({-2e-10, 0.0}, {2e-10, 0.0});
    const SCArcSegment2d arc({0.0, 0.0}, 1e-10, 0.0, Geometry::kTwoPi);
    const auto result = Geometry::IntersectExtended(line, arc, Geometry::SCExtensionPolicy::ExtendBoth, eps);
    ASSERT_EQ(result.pointCount, 2U);
    EXPECT_EQ(result.kind, Geometry::SCIntersectionKind2d::Point);
    EXPECT_TRUE(result.onFirstSegment);
    EXPECT_TRUE(result.onSecondSegment);
}

TEST(SegmentTest, ExtendedLineArcFlagsRequireAllReturnedPointsOnSegments)
{
    const SCLineSegment2d line({2.0, 0.0}, {0.5, 0.0});
    const SCArcSegment2d arc({0.0, 0.0}, 1.0, 0.0, Geometry::kPi / 2.0);

    const auto result = Geometry::IntersectExtended(line, arc, Geometry::SCExtensionPolicy::ExtendBoth);
    ASSERT_EQ(result.pointCount, 2U);
    EXPECT_EQ(result.kind, Geometry::SCIntersectionKind2d::Point);
    EXPECT_FALSE(result.onFirstSegment);
    EXPECT_FALSE(result.onSecondSegment);

    const auto reversed = Geometry::IntersectExtended(arc, line, Geometry::SCExtensionPolicy::ExtendBoth);
    ASSERT_EQ(reversed.pointCount, 2U);
    EXPECT_FALSE(reversed.onFirstSegment);
    EXPECT_FALSE(reversed.onSecondSegment);
}

TEST(SegmentTest, ExplicitToleranceExtendedArcArcUsesAngleTolerance)
{
    constexpr double eps = 1e-12;
    const SCArcSegment2d first({0.0, 0.0}, 1e-8, 0.0, Geometry::kPi / 3.0 - 1e-6);
    const SCArcSegment2d second({1e-8, 0.0}, 1e-8, 0.0, Geometry::kTwoPi);
    const auto result = Geometry::IntersectExtended(first, second, Geometry::SCExtensionPolicy::ExtendSecond, eps);
    ASSERT_EQ(result.pointCount, 1U);
    EXPECT_TRUE(result.onFirstSegment);
    EXPECT_TRUE(result.onSecondSegment);
}

TEST(SegmentTest, ExplicitToleranceLineSegmentIntersectionUsesSegmentLength)
{
    constexpr double eps = 1e-12;
    const Geometry::SCLine2d line = Geometry::SCLine2d::FromTwoPoints({-1.0, 0.0}, {1.0, 0.0});
    const SCLineSegment2d nearSegment({0.0, 5e-13}, {0.0, 1e-10 + 5e-13});
    const auto nearHit = Geometry::Intersect(line, nearSegment, eps);
    ASSERT_EQ(nearHit.pointCount, 1U);
    EXPECT_NEAR(nearHit.points[0].point.y, 0.0, eps);
    EXPECT_EQ(nearHit.points[0].parameterOnSecond, 0.0);
    const auto reversed = Geometry::Intersect(nearSegment, line, eps);
    ASSERT_EQ(reversed.pointCount, 1U);
    EXPECT_EQ(reversed.points[0].parameterOnFirst, 0.0);

    const SCLineSegment2d separated({0.0, 2e-12}, {0.0, 1e-10 + 2e-12});
    EXPECT_EQ(Geometry::Intersect(line, separated, eps).pointCount, 0U);
}
