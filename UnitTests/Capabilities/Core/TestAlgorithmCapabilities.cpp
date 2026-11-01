#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "Geometry.h"

using Geometry::SCIntersectionKind2d;
using Geometry::SCLine2d;
using Geometry::SCLine3d;
using Geometry::SCLineSegment2d;
using Geometry::SCLineSegment3d;
using Geometry::SCPoint2d;
using Geometry::SCPoint3d;
using Geometry::SCTangentPoints2d;
using Geometry::TangentPoints;

TEST(AlgorithmCapabilitiesTest, TangentPointsReturnsTwoPointsForExternalPoint)
{
    const Geometry::SCCircle2d circle(SCPoint2d{0.0, 0.0}, 5.0);
    const SCTangentPoints2d result = TangentPoints(SCPoint2d{10.0, 0.0}, circle);

    ASSERT_TRUE(result.IsValid());
    EXPECT_EQ(result.pointCount, 2U);
    EXPECT_TRUE(result.points[0].AlmostEquals(SCPoint2d{2.5, -2.5 * std::sqrt(3.0)}, 1e-12));
    EXPECT_TRUE(result.points[1].AlmostEquals(SCPoint2d{2.5, 2.5 * std::sqrt(3.0)}, 1e-12));
}

TEST(AlgorithmCapabilitiesTest, TangentPointsRejectInvalidActivePayload)
{
    SCTangentPoints2d result{};
    EXPECT_TRUE(result.IsValid());
    result.pointCount = 1;
    ASSERT_TRUE(result.IsValid());
    result.points[0].x = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(result.IsValid());
    result.points[0].x = 0.0;
    result.angles[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(result.IsValid());
    result.angles[0] = 0.0;
    result.pointInsideCircle = true;
    EXPECT_FALSE(result.IsValid());
    result.pointInsideCircle = false;
    result.pointCount = 3;
    EXPECT_FALSE(result.IsValid());
}

TEST(AlgorithmCapabilitiesTest, Line2dIntersectionFindsPointAndSegmentHit)
{
    const SCLine2d first = SCLine2d::FromTwoPoints(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0});
    const SCLine2d second = SCLine2d::FromTwoPoints(SCPoint2d{2.0, -2.0}, SCPoint2d{2.0, 2.0});
    const auto lineLine = Geometry::Intersect(first, second);
    ASSERT_TRUE(lineLine.HasIntersection());
    ASSERT_EQ(lineLine.pointCount, 1U);
    EXPECT_TRUE(lineLine.points[0].point.AlmostEquals(SCPoint2d{2.0, 0.0}, 1e-12));

    const SCLineSegment2d segment(SCPoint2d{2.0, -1.0}, SCPoint2d{2.0, 1.0});
    const auto lineSegment = Geometry::Intersect(first, segment);
    ASSERT_TRUE(lineSegment.HasIntersection());
    ASSERT_EQ(lineSegment.pointCount, 1U);
    EXPECT_TRUE(lineSegment.points[0].point.AlmostEquals(SCPoint2d{2.0, 0.0}, 1e-12));
}

TEST(AlgorithmCapabilitiesTest, IntersectExtendedPolicyNoneMatchesIntersectForCollinearLines)
{
    const SCLineSegment2d first{SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}};
    const SCLineSegment2d second{SCPoint2d{2.0, 0.0}, SCPoint2d{6.0, 0.0}};

    const auto expected = Geometry::Intersect(first, second);
    const auto actual = Geometry::IntersectExtended(first, second, Geometry::SCExtensionPolicy::None);

    ASSERT_TRUE(expected.HasIntersection());
    ASSERT_TRUE(actual.HasIntersection());
    EXPECT_EQ(actual.kind, expected.kind);
    EXPECT_EQ(actual.pointCount, expected.pointCount);
    EXPECT_FALSE(actual.infiniteOverlap);
    ASSERT_EQ(actual.pointCount, 2U);
    EXPECT_TRUE(actual.points[0].point.AlmostEquals(expected.points[0].point, 1e-12));
    EXPECT_TRUE(actual.points[1].point.AlmostEquals(expected.points[1].point, 1e-12));
    EXPECT_TRUE(actual.onFirstSegment);
    EXPECT_TRUE(actual.onSecondSegment);
}

TEST(AlgorithmCapabilitiesTest, IntersectionPointsRejectNonFiniteParameters)
{
    Geometry::SCIntersectionPoint2d point{{1.0, 2.0}, 0.25, 0.5};
    ASSERT_TRUE(point.IsValid());
    point.parameterOnFirst = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(point.IsValid());
    point.parameterOnFirst = 0.25;
    point.parameterOnSecond = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(point.IsValid());

    Geometry::SCPolylineIntersectionPoint2d polyline{};
    polyline.kind = SCIntersectionKind2d::Point;
    polyline.point = {1.0, 2.0};
    ASSERT_TRUE(polyline.IsValid());
    polyline.parameterOnFirstSegment = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(polyline.IsValid());
    polyline.parameterOnFirstSegment = 0.25;
    polyline.parameterOnSecondSegment = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(polyline.IsValid());
    polyline.parameterOnSecondSegment = 0.5;
    polyline.globalParameterOnFirst = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(polyline.IsValid());
    polyline.globalParameterOnFirst = 0.25;
    polyline.globalParameterOnSecond = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(polyline.IsValid());
}

TEST(AlgorithmCapabilitiesTest, LineProjectionRejectsUnrepresentableSquaredDistance)
{
    const SCLine3d line = SCLine3d::FromOriginAndDirection({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    const auto nearby = Geometry::ProjectPointToLine({0.0, 1e-200, 0.0}, line);
    ASSERT_TRUE(nearby.IsValid());
    EXPECT_TRUE(nearby.isOnLine);
    EXPECT_DOUBLE_EQ(nearby.distanceSquared, 0.0);

    const auto far = Geometry::ProjectPointToLine({0.0, 1e200, 0.0}, line);
    EXPECT_FALSE(far.IsValid());
    EXPECT_FALSE(far.isOnLine);
}

TEST(AlgorithmCapabilitiesTest, DefaultPrimitiveProjectionsAreInvalid)
{
    EXPECT_FALSE(Geometry::SCSegmentProjection3d{}.IsValid());
    EXPECT_FALSE(Geometry::SCLineProjection3d{}.IsValid());
    EXPECT_FALSE(Geometry::SCPlaneProjection3d{}.IsValid());
}

TEST(AlgorithmCapabilitiesTest, BodyProjectionSuccessRequiresSuccessfulFace)
{
    Geometry::SCBrepBodyProjection3d brep{};
    EXPECT_TRUE(brep.IsValid());
    brep.success = true;
    EXPECT_FALSE(brep.IsValid());
    brep.projection.success = true;
    EXPECT_TRUE(brep.IsValid());
    brep.projection.distanceSquared = -1.0;
    EXPECT_FALSE(brep.IsValid());

    Geometry::SCPolyhedronBodyProjection3d polyhedron{};
    EXPECT_TRUE(polyhedron.IsValid());
    polyhedron.success = true;
    EXPECT_FALSE(polyhedron.IsValid());
    polyhedron.projection.success = true;
    EXPECT_TRUE(polyhedron.IsValid());
    polyhedron.projection.distanceSquared = -1.0;
    EXPECT_FALSE(polyhedron.IsValid());
}

TEST(AlgorithmCapabilitiesTest, ProjectPointToLineSegment3dClampsAndProjects)
{
    const SCLineSegment3d segment{SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{2.0, 0.0, 0.0}};
    const auto projection = Geometry::ProjectPointToLineSegment(SCPoint3d{1.0, 0.0, 1.0}, segment, true);

    ASSERT_TRUE(projection.IsValid());
    EXPECT_DOUBLE_EQ(projection.parameter, 0.5);
    EXPECT_TRUE(projection.point.AlmostEquals(SCPoint3d{1.0, 0.0, 0.0}, 1e-12));
    EXPECT_DOUBLE_EQ(projection.distanceSquared, 1.0);
}
