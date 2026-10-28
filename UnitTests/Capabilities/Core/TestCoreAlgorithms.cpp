#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <limits>
#include <memory>

#include "Geometry.h"

using Geometry::SCArcDirection;
using Geometry::SCArcSegment2d;
using Geometry::SCAxisProjection2d;
using Geometry::SCAxisSample2d;
using Geometry::SCBox2d;
using Geometry::SCClosestPoints2d;
using Geometry::SCIntersectionKind2d;
using Geometry::SCLineSegment2d;
using Geometry::SCPoint2d;
using Geometry::SCPolygon2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::SCRingOrientation2d;
using Geometry::ISCSegment2d;
using Geometry::SCSegmentIntersection2d;
using Geometry::SCSegmentKind2;
using Geometry::SCSegmentProjection2d;
using Geometry::SCSegmentSplit2d;
using Geometry::SCSegmentTrim2d;
using Geometry::SCSnapResult2d;
using Geometry::SCVector2d;

TEST(CoreAlgorithmsTest, RingOrientationUsesAreaTolerance)
{
    const SCPolyline2d ring({{0.0, 0.0}, {1e-4, 0.0}, {1e-4, 5e-6}, {0.0, 5e-6}},
                            SCPolylineClosure::Closed);
    ASSERT_TRUE(ring.IsValid());
    EXPECT_EQ(Geometry::Orientation(ring), SCRingOrientation2d::CounterClockwise);
}

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

TEST(CoreAlgorithmsTest, ParameterAtLengthDispatchesThroughSegmentInterface)
{
    const SCLineSegment2d line({0.0, 0.0}, {4.0, 0.0});
    const SCArcSegment2d arc({0.0, 0.0}, 2.0, 0.0, Geometry::kPi * 0.5);
    const ISCSegment2d& genericLine = line;
    const ISCSegment2d& genericArc = arc;

    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericLine, 2.0), 0.5);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericLine, 6.0, true), 1.0);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericArc, arc.Length() * 0.5), 0.5);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericArc, arc.Length() * 2.0, true), 1.0);
}

TEST(CoreAlgorithmsTest, ParameterAtLengthAcceptsExplicitToleranceForSmallSegments)
{
    const SCLineSegment2d line({0.0, 0.0}, {1e-10, 0.0});
    const SCArcSegment2d arc({0.0, 0.0}, 1e-10, 0.0, Geometry::kPi * 0.5);
    const ISCSegment2d& genericLine = line;
    const ISCSegment2d& genericArc = arc;

    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericLine, 5e-11), 0.0);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericLine, 5e-11, false, 1e-15), 0.5);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericArc, arc.Length() * 0.5, false, 1e-15), 0.5);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericLine, 2e-10, true, 1e-15), 1.0);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericLine, 5e-11, false, 0.0), 0.0);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericLine, 5e-11, false,
                                                std::numeric_limits<double>::quiet_NaN()), 0.0);
    EXPECT_DOUBLE_EQ(Geometry::ParameterAtLength(genericLine, std::numeric_limits<double>::max(),
                                                false, 1e-15), 0.0);
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

TEST(CoreAlgorithmsTest, SmallCallerToleranceProjectsShortLine)
{
    constexpr double length = 1e-10;
    constexpr double eps = 1e-15;
    const SCLineSegment2d line({0.0, 0.0}, {length, 0.0});
    const SCPoint2d query{0.5 * length, 1e-11};
    EXPECT_EQ(ProjectPointToLineSegment(query, line, true).point, line.startPoint);

    const auto typed = ProjectPointToLineSegment(query, line, true, eps);
    const auto endpoints = ProjectPointToSegment(query, line.startPoint, line.endPoint, true, eps);
    for (const auto& projection : {typed, endpoints})
    {
        ASSERT_TRUE(projection.IsValid());
        EXPECT_TRUE(projection.point.AlmostEquals({0.5 * length, 0.0}, eps));
        EXPECT_NEAR(projection.parameter, 0.5, 1e-12);
        EXPECT_TRUE(projection.isOnSegment);
    }

    const auto beyond = ProjectPointToLineSegment({2.0 * length, 0.0}, line, true, eps);
    EXPECT_TRUE(beyond.point.AlmostEquals(line.endPoint, eps));
    EXPECT_FALSE(beyond.isOnSegment);
    EXPECT_FALSE(ProjectPointToLineSegment(query, line, true,
                                          std::numeric_limits<double>::infinity()).IsValid());
}

TEST(CoreAlgorithmsTest, SmallCallerToleranceProjectsArcAndGenericSegments)
{
    constexpr double radius = 1e-10;
    constexpr double eps = 1e-15;
    constexpr double nearAngle = 1e-6;
    const SCArcSegment2d arc({0.0, 0.0}, radius, 0.0, Geometry::kPi * 0.5);
    const SCPoint2d beforeStart{radius * std::cos(-nearAngle), radius * std::sin(-nearAngle)};
    const SCPoint2d afterEnd{radius * std::cos(Geometry::kPi * 0.5 + nearAngle),
                             radius * std::sin(Geometry::kPi * 0.5 + nearAngle)};

    const auto startProjection = ProjectPointToArcSegment(beforeStart, arc, true, eps);
    ASSERT_TRUE(startProjection.IsValid());
    EXPECT_TRUE(startProjection.point.AlmostEquals(arc.StartPoint(), eps));
    EXPECT_DOUBLE_EQ(startProjection.parameter, 0.0);
    EXPECT_TRUE(startProjection.isOnSegment);

    const auto endProjection = ProjectPointToArcSegment(afterEnd, arc, true, eps);
    ASSERT_TRUE(endProjection.IsValid());
    EXPECT_TRUE(endProjection.point.AlmostEquals(arc.EndPoint(), eps));
    EXPECT_DOUBLE_EQ(endProjection.parameter, 1.0);
    EXPECT_TRUE(endProjection.isOnSegment);

    const SCPoint2d farBeforeStart{radius * std::cos(-0.2), radius * std::sin(-0.2)};
    const auto farProjection = ProjectPointToArcSegment(farBeforeStart, arc, true, eps);
    EXPECT_TRUE(farProjection.point.AlmostEquals(arc.StartPoint(), eps));
    EXPECT_FALSE(farProjection.isOnSegment);

    const ISCSegment2d& genericArc = arc;
    EXPECT_TRUE(ProjectPointToSegment(beforeStart, genericArc, true, eps)
                    .point.AlmostEquals(arc.StartPoint(), eps));
    const SCLineSegment2d line({0.0, 0.0}, {radius, 0.0});
    const ISCSegment2d& genericLine = line;
    EXPECT_TRUE(ProjectPointToSegment({radius * 0.5, radius * 0.1}, genericLine, true, eps)
                    .point.AlmostEquals({radius * 0.5, 0.0}, eps));
    const SCArcSegment2d clockwise({0.0, 0.0}, radius, 0.0, -Geometry::kPi * 0.5);
    const SCPoint2d beforeClockwiseStart{radius * std::cos(nearAngle), radius * std::sin(nearAngle)};
    const auto clockwiseProjection = ProjectPointToArcSegment(beforeClockwiseStart, clockwise, true, eps);
    EXPECT_TRUE(clockwiseProjection.point.AlmostEquals(clockwise.StartPoint(), eps));
    EXPECT_TRUE(clockwiseProjection.isOnSegment);
    const auto unbounded = ProjectPointToArcSegment(beforeStart, arc, false, eps);
    EXPECT_TRUE(unbounded.point.AlmostEquals(beforeStart, eps));
    EXPECT_GT(unbounded.parameter, 1.0);
    EXPECT_FALSE(ProjectPointToArcSegment(beforeStart, arc, true, 0.0).IsValid());
}

TEST(CoreAlgorithmsTest, ExplicitToleranceComputesShortSegmentTangentsAndNormals)
{
    constexpr double epsilon = 1e-15;
    const SCLineSegment2d line({0.0, 0.0}, {1e-10, 0.0});
    const SCArcSegment2d arc({0.0, 0.0}, 1e-10, 0.0, Geometry::kPi * 0.5);
    const ISCSegment2d& lineBase = line;
    const ISCSegment2d& arcBase = arc;

    EXPECT_EQ(TangentAt(line, 0.5), SCVector2d{});
    EXPECT_EQ(TangentAt(arc, 0.0), SCVector2d{});
    EXPECT_EQ(TangentAt(line, 0.5, epsilon), (SCVector2d{1.0, 0.0}));
    EXPECT_EQ(NormalAt(line, 0.5, epsilon), (SCVector2d{0.0, 1.0}));
    EXPECT_EQ(TangentAt(lineBase, 0.5, epsilon), (SCVector2d{1.0, 0.0}));
    EXPECT_EQ(NormalAt(lineBase, 0.5, epsilon), (SCVector2d{0.0, 1.0}));
    EXPECT_TRUE(TangentAt(arc, 0.0, epsilon).AlmostEquals({0.0, 1.0}, epsilon));
    EXPECT_TRUE(NormalAt(arc, 0.0, epsilon).AlmostEquals({-1.0, 0.0}, epsilon));
    EXPECT_TRUE(TangentAt(arcBase, 0.0, epsilon).AlmostEquals({0.0, 1.0}, epsilon));
    EXPECT_TRUE(NormalAt(arcBase, 0.0, epsilon).AlmostEquals({-1.0, 0.0}, epsilon));
    const SCArcSegment2d clockwise({0.0, 0.0}, 1e-10, 0.0, -Geometry::kPi * 0.5);
    EXPECT_TRUE(TangentAt(clockwise, 0.0, epsilon).AlmostEquals({0.0, -1.0}, epsilon));
    EXPECT_EQ(TangentAt(line, 0.5, 1e-10), SCVector2d{});
    const SCArcSegment2d shortArc({0.0, 0.0}, 1e-10, 0.0, 0.1);
    EXPECT_EQ(TangentAt(shortArc, 0.5, 2e-11), SCVector2d{});
    EXPECT_EQ(TangentAt(line, 0.5, 0.0), SCVector2d{});
    EXPECT_EQ(NormalAt(arc, 0.0, -1.0), SCVector2d{});
    EXPECT_EQ(TangentAt(line, std::numeric_limits<double>::infinity(), epsilon), SCVector2d{});
    EXPECT_EQ(TangentAt(arc, std::numeric_limits<double>::max(), epsilon), SCVector2d{});
}

TEST(CoreAlgorithmsTest, ExplicitToleranceProjectsAndOrientsShortSweepArc)
{
    constexpr double epsilon = 1e-9;
    const SCArcSegment2d arc({0.0, 0.0}, 1e6, 0.0, 1e-10);
    ASSERT_TRUE(arc.IsValid(epsilon));
    const auto projection = Geometry::ProjectPointToArcSegment({1e6, 5e-5}, arc, true, epsilon);
    ASSERT_TRUE(projection.IsValid());
    EXPECT_NEAR(projection.parameter, 0.5, 1e-6);
    EXPECT_NEAR(projection.point.y, 5e-5, 1e-12);
    EXPECT_NEAR(Geometry::TangentAt(arc, 0.5, epsilon).y, 1.0, 1e-12);
    EXPECT_NEAR(Geometry::NormalAt(arc, 0.5, epsilon).x, -1.0, 1e-12);
}

TEST(CoreAlgorithmsTest, CoversCurrentCapabilities)
{
    const SCLineSegment2d line(SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0});
    const SCArcSegment2d arc(SCPoint2d{0.0, 0.0}, 2.0, 0.0, Geometry::kPi * 0.5);

    EXPECT_NEAR(Distance(SCPoint2d{1.0, 1.0}, line), 1.0, 1e-12);
    EXPECT_NEAR(Distance(SCPoint2d{1.0, 1.0}, arc), std::abs(std::sqrt(2.0) - 2.0), 1e-12);
    EXPECT_NEAR(line.Length(), 4.0, 1e-12);
    EXPECT_TRUE(arc.Bounds().AlmostEquals(SCBox2d::FromMinMax(SCPoint2d{0.0, 0.0}, SCPoint2d{2.0, 2.0}), 1e-12));

    const SCSegmentProjection2d lineProjection = ProjectPointToLineSegment(SCPoint2d{1.0, 2.0}, line, true);
    EXPECT_TRUE(lineProjection.point.AlmostEquals(SCPoint2d{1.0, 0.0}, 1e-12));
    EXPECT_NEAR(lineProjection.parameter, 0.25, 1e-12);

    const SCSegmentProjection2d arcProjection = ProjectPointToArcSegment(SCPoint2d{1.0, 1.0}, arc, true);
    EXPECT_TRUE(arcProjection.point.AlmostEquals(SCPoint2d{std::sqrt(2.0), std::sqrt(2.0)}, 1e-12));

    EXPECT_TRUE(line.PointAtLength(2.0, true).AlmostEquals(SCPoint2d{2.0, 0.0}, 1e-12));
    EXPECT_TRUE(TangentAt(line, 0.5).AlmostEquals(SCVector2d{1.0, 0.0}, 1e-12));
    EXPECT_TRUE(NormalAt(line, 0.5).AlmostEquals(SCVector2d{0.0, 1.0}, 1e-12));

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

    const SCPolyline2d ccwRing({SCPoint2d{0.0, 0.0}, SCPoint2d{4.0, 0.0}, SCPoint2d{4.0, 4.0}, SCPoint2d{0.0, 4.0}},
                             SCPolylineClosure::Closed);
    const SCPolyline2d holeRing(
        {SCPoint2d{1.0, 1.0}, SCPoint2d{1.0, 3.0}, SCPoint2d{3.0, 3.0}, SCPoint2d{3.0, 1.0}}, SCPolylineClosure::Closed);
    const SCPolygon2d polygon(ccwRing, {holeRing});
    EXPECT_NEAR(polygon.Area(), 12.0, 1e-12);
    EXPECT_TRUE(polygon.Centroid().AlmostEquals(SCPoint2d{2.0, 2.0}, 1e-12));
    ASSERT_EQ(Orientation(ccwRing), SCRingOrientation2d::CounterClockwise);
    ASSERT_TRUE(IsCounterClockwise(ccwRing));
    ASSERT_TRUE(IsClockwise(holeRing));

    const SCLineSegment2d reversedLine = Reverse(line);
    EXPECT_TRUE(reversedLine.startPoint.AlmostEquals(line.endPoint, 1e-12));
    const SCArcSegment2d reversedArc = Reverse(arc);
    EXPECT_TRUE(reversedArc.StartPoint().AlmostEquals(arc.EndPoint(), 1e-12));
    const SCPolyline2d closedOpen =
        Close(SCPolyline2d({SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 0.0}, SCPoint2d{1.0, 1.0}}, SCPolylineClosure::Open));
    ASSERT_TRUE(closedOpen.IsClosed());
    const SCPolyline2d openArcPath(
        {std::make_shared<SCLineSegment2d>(SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 0.0}),
         std::make_shared<SCArcSegment2d>(
             SCPoint2d{1.0, 1.0}, 1.0, -Geometry::kPi * 0.5, 0.0, SCArcDirection::CounterClockwise)},
        SCPolylineClosure::Open);
    const SCPolyline2d closedArcPath = Close(openArcPath);
    ASSERT_TRUE(closedArcPath.IsClosed());
    ASSERT_EQ(closedArcPath.SegmentAt(0)->Kind(), SCSegmentKind2::Line);
    ASSERT_EQ(closedArcPath.SegmentAt(1)->Kind(), SCSegmentKind2::Arc);
    ASSERT_EQ(closedArcPath.SegmentAt(2)->Kind(), SCSegmentKind2::Line);

    const SCAxisSample2d sample = SampleAxis(line, 0.5);
    EXPECT_TRUE(sample.point.AlmostEquals(SCPoint2d{2.0, 0.0}, 1e-12));
    EXPECT_TRUE(sample.tangent.AlmostEquals(SCVector2d{1.0, 0.0}, 1e-12));
    const SCAxisProjection2d axisProjection = ProjectPointToAxis(SCPoint2d{1.0, 1.0}, line);
    EXPECT_TRUE(axisProjection.projection.point.AlmostEquals(SCPoint2d{1.0, 0.0}, 1e-12));

    const SCSegmentSplit2d split = SplitSegment(line, 0.5);
    ASSERT_TRUE(split.success);
    ASSERT_EQ(split.first->Kind(), SCSegmentKind2::Line);
    ASSERT_EQ(split.second->Kind(), SCSegmentKind2::Line);

    const SCSegmentTrim2d trim = TrimSegment(arc, 0.25, 0.75);
    ASSERT_TRUE(trim.success);
    ASSERT_EQ(trim.segment->Kind(), SCSegmentKind2::Arc);

    const std::unique_ptr<ISCSegment2d> lineClone = line.Clone();
    const std::unique_ptr<ISCSegment2d> arcClone = arc.Clone();
    const std::array<const ISCSegment2d*, 2> snapSegments{lineClone.get(), arcClone.get()};
    const SCSnapResult2d snap = SnapPointToSegments(SCPoint2d{1.0, 1.0}, snapSegments, 2.0);
    ASSERT_TRUE(snap.snapped);
}
