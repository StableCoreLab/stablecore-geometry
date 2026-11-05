#include <gtest/gtest.h>
#include <limits>
#include "Core/Intersection.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Support/Epsilon.h"

using Geometry::SCArcSegment2d;
using Geometry::SCClosestPoints2d;
using Geometry::SCIntersectionKind2d;
using Geometry::SCLineSegment2d;
using Geometry::SCPoint2d;
using Geometry::SCSegmentIntersection2d;

TEST(CoreAlgorithmsTest, ClosestPointsKeepsSubEpsilonSquaredDistanceImprovement)
{
    const SCLineSegment2d first({0.0, 0.0}, {1.0, 0.0});
    const SCLineSegment2d second({0.0, 2e-5}, {1.0, 1e-5});
    const auto closest = ClosestPoints(first, second);
    ASSERT_TRUE(closest.IsValid());
    EXPECT_NEAR(closest.distanceSquared, 1e-10, 1e-15);
    EXPECT_NEAR(closest.parameterOnFirst, 1.0, 1e-9);
    EXPECT_NEAR(closest.parameterOnSecond, 1.0, 1e-9);
}

TEST(CoreAlgorithmsTest, SmallCallerToleranceFindsShortSegmentCrossingAndOverlap)
{
    constexpr double scale = 1e-10;
    constexpr double eps = 1e-15;
    const SCLineSegment2d horizontal({0.0, 0.0}, {4.0 * scale, 0.0});
    const SCLineSegment2d vertical({scale, -scale}, {scale, scale});
    const auto crossing = Intersect(horizontal, vertical, eps);
    ASSERT_EQ(crossing.kind, SCIntersectionKind2d::Point);
    ASSERT_EQ(crossing.pointCount, 1U);
    EXPECT_TRUE(crossing.points[0].point.AlmostEquals({scale, 0.0}, eps));
    EXPECT_NEAR(crossing.points[0].parameterOnFirst, 0.25, 1e-12);
    EXPECT_NEAR(crossing.points[0].parameterOnSecond, 0.5, 1e-12);

    const SCLineSegment2d shifted({scale, 0.0}, {5.0 * scale, 0.0});
    const auto overlap = Intersect(horizontal, shifted, eps);
    ASSERT_EQ(overlap.kind, SCIntersectionKind2d::Overlap);
    ASSERT_EQ(overlap.pointCount, 2U);
    EXPECT_TRUE(overlap.points[0].point.AlmostEquals({scale, 0.0}, eps));
    EXPECT_TRUE(overlap.points[1].point.AlmostEquals({4.0 * scale, 0.0}, eps));
}

TEST(CoreAlgorithmsTest, IntersectsAndFindsClosestBasicSegments)
{
    const SCLineSegment2d line(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0});
    const SCArcSegment2d arc(SCPoint2d{0.0, 0.0}, 2.0, 0.0, Geometry::kPi * 0.5);

    const SCLineSegment2d vertical(SCPoint2d{2.0, -1.0}, SCPoint2d{2.0, 1.0});
    const SCSegmentIntersection2d lineLineIntersection = Intersect(line, vertical);
    ASSERT_EQ(lineLineIntersection.kind, SCIntersectionKind2d::Point);
    ASSERT_EQ(lineLineIntersection.pointCount, 1);
    EXPECT_TRUE(lineLineIntersection.points[0].point.AlmostEquals(SCPoint2d{2.0, 0.0}, 1e-12));

    const SCSegmentIntersection2d lineArcIntersection = Intersect(line, arc);
    ASSERT_TRUE(lineArcIntersection.HasIntersection());

    const SCArcSegment2d secondArc(SCPoint2d{2.0, 0.0}, 2.0, Geometry::kPi, -Geometry::kPi * 0.5);
    const SCSegmentIntersection2d arcArcIntersection = Intersect(arc, secondArc);
    ASSERT_TRUE(arcArcIntersection.HasIntersection());

    const SCClosestPoints2d closest = ClosestPoints(line, secondArc);
    ASSERT_TRUE(closest.IsValid());
}

TEST(CoreAlgorithmsTest, ClosestPointsResultRejectsInvalidPayload)
{
    EXPECT_FALSE(SCClosestPoints2d{}.IsValid());
    SCClosestPoints2d result{{0.0, 0.0}, {1.0, 0.0}, 0.0, 1.0, 1.0};
    ASSERT_TRUE(result.IsValid());
    result.parameterOnFirst = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(result.IsValid());
    result.parameterOnFirst = 0.0;
    result.distanceSquared = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(result.IsValid());
}
