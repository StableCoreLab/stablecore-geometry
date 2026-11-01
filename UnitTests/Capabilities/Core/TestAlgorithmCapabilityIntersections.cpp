#include <gtest/gtest.h>

#include <cmath>

#include "Geometry.h"

using Geometry::SCIntersectionKind2d;
using Geometry::SCArcSegment2d;
using Geometry::SCLineSegment2d;
using Geometry::SCPoint2d;

TEST(AlgorithmCapabilitiesTest, IntersectExtendedLineArcFindsExtendedHit)
{
    const SCLineSegment2d lineSegment{SCPoint2d{3.0, 3.0}, SCPoint2d{4.0, 4.0}};
    const SCArcSegment2d arc{SCPoint2d{0.0, 0.0}, std::sqrt(8.0), 0.0, Geometry::kPi / 2.0};

    const auto withoutExtension = Geometry::IntersectExtended(lineSegment, arc, Geometry::SCExtensionPolicy::None);
    EXPECT_FALSE(withoutExtension.HasIntersection());

    const auto extended = Geometry::IntersectExtended(lineSegment, arc, Geometry::SCExtensionPolicy::ExtendFirst);
    ASSERT_TRUE(extended.HasIntersection());
    EXPECT_EQ(extended.kind, SCIntersectionKind2d::Point);
    ASSERT_EQ(extended.pointCount, 1U);
    EXPECT_TRUE(extended.points[0].point.AlmostEquals(SCPoint2d{2.0, 2.0}, 1e-12));
    EXPECT_FALSE(extended.onFirstSegment);
    EXPECT_TRUE(extended.onSecondSegment);
}

TEST(AlgorithmCapabilitiesTest, IntersectExtendedLineArcUsesCallerEpsilonForArcBoundary)
{
    const double delta = 5e-7;
    const double eps = 1e-6;
    const double angle = Geometry::kPi / 2.0 + delta;
    const SCPoint2d tangentPoint{std::cos(angle), std::sin(angle)};
    const Geometry::SCVector2d tangentDirection{-std::sin(angle), std::cos(angle)};
    const SCLineSegment2d lineSegment{tangentPoint - tangentDirection * 0.5, tangentPoint + tangentDirection * 0.5};
    const SCArcSegment2d arc{SCPoint2d{0.0, 0.0}, 1.0, 0.0, Geometry::kPi / 2.0};

    const auto strict = Geometry::IntersectExtended(lineSegment, arc, Geometry::SCExtensionPolicy::None, 1e-9);
    EXPECT_FALSE(strict.HasIntersection());

    const auto relaxed = Geometry::IntersectExtended(lineSegment, arc, Geometry::SCExtensionPolicy::None, eps);
    ASSERT_TRUE(relaxed.HasIntersection());
    EXPECT_EQ(relaxed.kind, SCIntersectionKind2d::Tangent);
    ASSERT_EQ(relaxed.pointCount, 1U);
    EXPECT_TRUE(relaxed.points[0].point.AlmostEquals(tangentPoint, 1e-12));
    EXPECT_TRUE(relaxed.onFirstSegment);
    EXPECT_TRUE(relaxed.onSecondSegment);
}

TEST(AlgorithmCapabilitiesTest, ArcArcExtendedIntersectionHonorsPerSideExtension)
{
    const SCArcSegment2d first{SCPoint2d{0.0, 0.0}, 5.0, 0.0, Geometry::kPi / 4.0};
    const SCArcSegment2d second{SCPoint2d{0.0, 0.0}, 5.0, Geometry::kPi / 2.0, Geometry::kPi / 6.0};

    const auto withoutExtension = Geometry::IntersectExtended(first, second, Geometry::SCExtensionPolicy::None);
    EXPECT_FALSE(withoutExtension.HasIntersection());

    const auto extendFirst = Geometry::IntersectExtended(first, second, Geometry::SCExtensionPolicy::ExtendFirst);
    ASSERT_TRUE(extendFirst.HasIntersection());
    EXPECT_EQ(extendFirst.kind, SCIntersectionKind2d::Overlap);
    ASSERT_EQ(extendFirst.pointCount, 2U);
    EXPECT_TRUE(extendFirst.points[0].point.AlmostEquals(SCPoint2d{0.0, 5.0}, 1e-12));
    EXPECT_TRUE(extendFirst.points[1].point.AlmostEquals(second.EndPoint(), 1e-12));
    EXPECT_FALSE(extendFirst.onFirstSegment);
    EXPECT_TRUE(extendFirst.onSecondSegment);
}
