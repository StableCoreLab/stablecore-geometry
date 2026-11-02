#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "Core/Metrics.h"
#include "Core/Projection.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCPolyline2d.h"

TEST(Measure2dTest, PointSegmentDistance)
{
    const Geometry::SCLineSegment2d segment({0.0, 0.0}, {2.0, 0.0});
    EXPECT_NEAR(Geometry::DistanceSquared(Geometry::SCPoint2d{1.0, 2.0}, segment), 4.0, 1e-12);
    EXPECT_NEAR(Geometry::Distance(Geometry::SCPoint2d{1.0, 2.0}, segment), 2.0, 1e-12);
}

TEST(Measure2dTest, PointSegmentDistancePreservesFirstOrderRange)
{
    const Geometry::SCLineSegment2d line({0.0, 0.0}, {1.0, 0.0});
    const Geometry::ISCSegment2d& baseLine = line;
    EXPECT_DOUBLE_EQ(Geometry::Distance({0.5, 1e200}, line), 1e200);
    EXPECT_DOUBLE_EQ(Geometry::Distance({0.5, 1e200}, baseLine), 1e200);
    EXPECT_DOUBLE_EQ(Geometry::Distance({0.5, 1e-200}, line), 1e-200);

    const Geometry::SCArcSegment2d arc({0.0, 0.0}, 1.0, 0.0, std::numbers::pi_v<double> / 2.0);
    const Geometry::ISCSegment2d& baseArc = arc;
    EXPECT_DOUBLE_EQ(Geometry::Distance({1e200, 0.0}, arc), 1e200);
    EXPECT_DOUBLE_EQ(Geometry::Distance({1e200, 0.0}, baseArc), 1e200);
}

TEST(Measure2dTest, LineProjectionHandlesUnrepresentableDirectionSquare)
{
    const Geometry::SCLineSegment2d longLine({0.0, 0.0}, {1e200, 0.0});
    const auto longProjection = Geometry::ProjectPointToLineSegment({5e199, 1.0}, longLine, true);
    const auto longExplicit = Geometry::ProjectPointToLineSegment({5e199, 1.0}, longLine, true, 1e-9);
    ASSERT_TRUE(longProjection.IsValid());
    ASSERT_TRUE(longExplicit.IsValid());
    EXPECT_NEAR(longProjection.parameter, 0.5, 1e-12);
    EXPECT_NEAR(longExplicit.parameter, 0.5, 1e-12);
    EXPECT_DOUBLE_EQ(longProjection.distanceSquared, 1.0);
    EXPECT_DOUBLE_EQ(Geometry::Distance({5e199, 1.0}, longLine), 1.0);

    const Geometry::SCLineSegment2d shortLine({0.0, 0.0}, {1e-200, 0.0});
    const auto shortExplicit = Geometry::ProjectPointToLineSegment({5e-201, 1e-200}, shortLine, true, 1e-201);
    ASSERT_TRUE(shortExplicit.IsValid());
    EXPECT_NEAR(shortExplicit.parameter, 0.5, 1e-12);
    EXPECT_DOUBLE_EQ(shortExplicit.point.x, 5e-201);
}

TEST(Measure2dTest, LineProjectionRecoversOverflowingDotProduct)
{
    const double length = std::ldexp(1.0, 500);
    const double coordinate = std::ldexp(1.0, 700);
    const Geometry::SCLineSegment2d line({0.0, 0.0}, {length, 0.0});
    const auto projection = Geometry::ProjectPointToLineSegment({coordinate, 0.0}, line, false);
    const auto explicitProjection = Geometry::ProjectPointToLineSegment({coordinate, 0.0}, line, false, 1e-9);
    ASSERT_TRUE(projection.IsValid());
    ASSERT_TRUE(explicitProjection.IsValid());
    EXPECT_DOUBLE_EQ(projection.parameter, std::ldexp(1.0, 200));
    EXPECT_DOUBLE_EQ(explicitProjection.parameter, projection.parameter);
    EXPECT_DOUBLE_EQ(projection.point.x, coordinate);
    EXPECT_DOUBLE_EQ(explicitProjection.point.x, coordinate);
    EXPECT_DOUBLE_EQ(projection.distanceSquared, 0.0);
}

TEST(Measure2dTest, LineProjectionPreservesParameterWithSubnormalDirectionSquare)
{
    const double length = std::ldexp(1.0, -530);
    const double coordinate = std::ldexp(1.0, -545);
    const Geometry::SCLineSegment2d line({0.0, 0.0}, {length, 0.0});
    const auto projection = Geometry::ProjectPointToLineSegment(
        {coordinate, 0.0}, line, true, std::ldexp(1.0, -550));
    ASSERT_TRUE(projection.IsValid());
    EXPECT_DOUBLE_EQ(projection.parameter, std::ldexp(1.0, -15));
    EXPECT_DOUBLE_EQ(projection.point.x, coordinate);
}

TEST(Measure2dTest, ArcProjectionClampsToNearestEndpointAcrossAngleSeam)
{
    for (const double sign : {1.0, -1.0})
    {
        const Geometry::SCArcSegment2d arc({0.0, 0.0}, 1.0, 0.0,
                                           sign * std::numbers::pi_v<double> / 2.0);
        const Geometry::SCPoint2d query{2.0, -sign};
        const auto projection = Geometry::ProjectPointToArcSegment(query, arc, true);
        const auto explicitProjection = Geometry::ProjectPointToArcSegment(query, arc, true, 1e-9);
        ASSERT_TRUE(projection.IsValid());
        ASSERT_TRUE(explicitProjection.IsValid());
        EXPECT_DOUBLE_EQ(projection.parameter, 0.0);
        EXPECT_DOUBLE_EQ(projection.parameter, explicitProjection.parameter);
        EXPECT_TRUE(projection.point.AlmostEquals(arc.StartPoint(), 1e-12));
        EXPECT_NEAR(Geometry::Distance(query, arc), std::sqrt(2.0), 1e-12);
        EXPECT_NEAR(Geometry::Distance(query, static_cast<const Geometry::ISCSegment2d&>(arc)),
                    std::sqrt(2.0), 1e-12);
        const Geometry::SCPoint2d endQuery{-1.0, 2.0 * sign};
        EXPECT_DOUBLE_EQ(Geometry::ProjectPointToArcSegment(endQuery, arc, true).parameter, 1.0);
        EXPECT_DOUBLE_EQ(Geometry::ProjectPointToArcSegment(endQuery, arc, true, 1e-9).parameter, 1.0);
    }
}

TEST(Measure2dTest, PolylineBounds)
{
    const Geometry::SCPolyline2d polyline(
        {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}}, Geometry::SCPolylineClosure::Closed);
    const Geometry::SCBox2d bounds = polyline.Bounds();
    EXPECT_NEAR(bounds.MinPoint().x, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MinPoint().y, 0.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().x, 2.0, 1e-12);
    EXPECT_NEAR(bounds.MaxPoint().y, 2.0, 1e-12);
}
