#include <array>
#include <gtest/gtest.h>
#include <cassert>
#include <cmath>
#include <memory>
#include <numbers>

#include "sdk/Geometry.h"
#include "support/GTestCompat.h"
#include "support/GeometryTestSupport.h"

using geometry::ArcDirection;
using geometry::SegmentKind2;
using geometry::sdk::ArcSegment2d;
using geometry::sdk::AxisProjection2d;
using geometry::sdk::AxisSample2d;
using geometry::sdk::Box2d;
using geometry::sdk::ClosestPoints2d;
using geometry::sdk::IntersectionKind2d;
using geometry::sdk::LineSegment2d;
using geometry::sdk::Point2d;
using geometry::sdk::Polygon2d;
using geometry::sdk::Polyline2d;
using geometry::sdk::PolylineClosure;
using geometry::sdk::RingOrientation2d;
using geometry::sdk::Segment2d;
using geometry::sdk::SegmentIntersection2d;
using geometry::sdk::SegmentProjection2d;
using geometry::sdk::SegmentSplit2d;
using geometry::sdk::SegmentTrim2d;
using geometry::sdk::SnapResult2d;
using geometry::sdk::Vector2d;

TEST(SdkAlgorithmsTest, CoversCurrentCapabilities)
{
    const LineSegment2d line(Point2d{0.0, 0.0}, Point2d{4.0, 0.0});
    const ArcSegment2d arc(Point2d{0.0, 0.0}, 2.0, 0.0, std::numbers::pi_v<double> * 0.5);

    GEOMETRY_TEST_ASSERT_NEAR(geometry::sdk::Distance(Point2d{1.0, 1.0}, line), 1.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(
        geometry::sdk::Distance(Point2d{1.0, 1.0}, arc),
        std::abs(std::sqrt(2.0) - 2.0),
        1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(line.Length(), 4.0, 1e-12);
    GEOMETRY_TEST_ASSERT_BOX_NEAR(arc.Bounds(), Box2d::FromMinMax(Point2d{0.0, 0.0}, Point2d{2.0, 2.0}), 1e-12);

    const SegmentProjection2d lineProjection =
        geometry::sdk::ProjectPointToLineSegment(Point2d{1.0, 2.0}, line, true);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(lineProjection.point, (Point2d{1.0, 0.0}), 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(lineProjection.parameter, 0.25, 1e-12);

    const SegmentProjection2d arcProjection =
        geometry::sdk::ProjectPointToArcSegment(Point2d{1.0, 1.0}, arc, true);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(
        arcProjection.point,
        (Point2d{std::sqrt(2.0), std::sqrt(2.0)}),
        1e-12);

    GEOMETRY_TEST_ASSERT_POINT_NEAR(
        line.PointAtLength(2.0, true),
        (Point2d{2.0, 0.0}),
        1e-12);
    GEOMETRY_TEST_ASSERT_VECTOR_NEAR(
        geometry::sdk::TangentAt(line, 0.5),
        (Vector2d{1.0, 0.0}),
        1e-12);
    GEOMETRY_TEST_ASSERT_VECTOR_NEAR(
        geometry::sdk::NormalAt(line, 0.5),
        (Vector2d{0.0, 1.0}),
        1e-12);

    const LineSegment2d vertical(Point2d{2.0, -1.0}, Point2d{2.0, 1.0});
    const SegmentIntersection2d lineLineIntersection = geometry::sdk::Intersect(line, vertical);
    assert(lineLineIntersection.kind == IntersectionKind2d::Point);
    assert(lineLineIntersection.pointCount == 1);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(lineLineIntersection.points[0].point, (Point2d{2.0, 0.0}), 1e-12);

    const SegmentIntersection2d lineArcIntersection = geometry::sdk::Intersect(line, arc);
    assert(lineArcIntersection.HasIntersection());

    const ArcSegment2d secondArc(Point2d{2.0, 0.0}, 2.0, std::numbers::pi_v<double>, -std::numbers::pi_v<double> * 0.5);
    const SegmentIntersection2d arcArcIntersection = geometry::sdk::Intersect(arc, secondArc);
    assert(arcArcIntersection.HasIntersection());

    const ClosestPoints2d closest = geometry::sdk::ClosestPoints(line, secondArc);
    assert(closest.IsValid());

    const Polyline2d ccwRing(
        {Point2d{0.0, 0.0}, Point2d{4.0, 0.0}, Point2d{4.0, 4.0}, Point2d{0.0, 4.0}},
        PolylineClosure::Closed);
    const Polyline2d holeRing = geometry::test::MakeSdkRectangleHoleRing(Point2d{1.0, 1.0}, Point2d{3.0, 3.0});
    const Polygon2d polygon(ccwRing, {holeRing});
    GEOMETRY_TEST_ASSERT_NEAR(polygon.Area(), 12.0, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(geometry::sdk::Centroid(polygon), (Point2d{2.0, 2.0}), 1e-12);
    assert(geometry::sdk::Orientation(ccwRing) == RingOrientation2d::CounterClockwise);
    assert(geometry::sdk::IsCounterClockwise(ccwRing));
    assert(geometry::sdk::IsClockwise(holeRing));

    const LineSegment2d reversedLine = geometry::sdk::Reverse(line);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(reversedLine.startPoint, line.endPoint, 1e-12);
    const ArcSegment2d reversedArc = geometry::sdk::Reverse(arc);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(reversedArc.StartPoint(), arc.EndPoint(), 1e-12);
    const Polyline2d closedOpen = geometry::sdk::Close(Polyline2d({Point2d{0.0, 0.0}, Point2d{1.0, 0.0}, Point2d{1.0, 1.0}}, PolylineClosure::Open));
    assert(closedOpen.IsClosed());

    const AxisSample2d sample = geometry::sdk::SampleAxis(line, 0.5);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(sample.point, (Point2d{2.0, 0.0}), 1e-12);
    GEOMETRY_TEST_ASSERT_VECTOR_NEAR(sample.tangent, (Vector2d{1.0, 0.0}), 1e-12);
    const AxisProjection2d axisProjection = geometry::sdk::ProjectPointToAxis(Point2d{1.0, 1.0}, line);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(axisProjection.projection.point, (Point2d{1.0, 0.0}), 1e-12);

    const SegmentSplit2d split = geometry::sdk::SplitSegment(line, 0.5);
    assert(split.success);
    assert(split.first->Kind() == SegmentKind2::Line);
    assert(split.second->Kind() == SegmentKind2::Line);

    const SegmentTrim2d trim = geometry::sdk::TrimSegment(arc, 0.25, 0.75);
    assert(trim.success);
    assert(trim.segment->Kind() == SegmentKind2::Arc);

    const std::unique_ptr<Segment2d> lineClone = line.Clone();
    const std::unique_ptr<Segment2d> arcClone = arc.Clone();
    const std::array<const Segment2d*, 2> snapSegments{lineClone.get(), arcClone.get()};
    const SnapResult2d snap = geometry::sdk::SnapPointToSegments(Point2d{1.0, 1.0}, snapSegments, 2.0);
    assert(snap.snapped);
}


