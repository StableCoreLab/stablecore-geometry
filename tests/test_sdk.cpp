#include <cassert>
#include <cmath>
#include <memory>
#include <numbers>

#include "sdk/Geometry.h"
#include "support/GeometryTestSupport.h"

using geometry::sdk::Contains;
using geometry::sdk::Distance;
using geometry::sdk::ArcSegment2d;
using geometry::sdk::Box2d;
using geometry::sdk::Intersects;
using geometry::sdk::LineSegment2d;
using geometry::sdk::Point2d;
using geometry::sdk::Polygon2d;
using geometry::sdk::ProjectPointToSegment;
using geometry::sdk::Polyline2d;
using geometry::sdk::PolylineClosure;
using geometry::sdk::Segment2d;
using geometry::sdk::Vector2d;

int main()
{
    const Point2d a = Point2d::FromXY(0.0, 0.0);
    const Point2d b = Point2d::FromXY(3.0, 4.0);
    const Vector2d offset = b - a;

    GEOMETRY_TEST_ASSERT_NEAR(Distance(a, b), 5.0, 1e-12);
    assert(offset == Vector2d(Vector2d{3.0, 4.0}));
    GEOMETRY_TEST_ASSERT_NEAR(offset.LengthSquared(), 25.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(offset.Length(), 5.0, 1e-12);
    assert(a + offset == b);
    assert(b - offset == a);

    const LineSegment2d line = LineSegment2d::FromEndpoints(a, b);
    assert(line.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(line.Length(), 5.0, 1e-12);
    const Point2d lineMidpoint{1.5, 2.0};
    GEOMETRY_TEST_ASSERT_POINT_NEAR(line.PointAt(0.5), lineMidpoint, 1e-12);
    GEOMETRY_TEST_ASSERT_BOX_NEAR(
        line.Bounds(),
        Box2d::FromMinMax(Point2d{0.0, 0.0}, Point2d{3.0, 4.0}),
        1e-12);
    assert(line.DebugString().find("LineSegment2d{start=") == 0);
    assert(line.Kind() == geometry::SegmentKind2::Line);

    const ArcSegment2d arc = ArcSegment2d::FromCenterRadiusStartSweep(
        Point2d{0.0, 0.0},
        1.0,
        0.0,
        std::numbers::pi_v<double> * 0.5);
    assert(arc.IsValid());
    assert(arc.Direction() == geometry::ArcDirection::CounterClockwise);
    GEOMETRY_TEST_ASSERT_NEAR(arc.Length(), std::numbers::pi_v<double> * 0.5, 1e-12);
    const Point2d arcStart{1.0, 0.0};
    const Point2d arcEnd{0.0, 1.0};
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arc.StartPoint(), arcStart, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arc.EndPoint(), arcEnd, 1e-12);
    GEOMETRY_TEST_ASSERT_BOX_NEAR(
        arc.Bounds(),
        Box2d::FromMinMax(Point2d{0.0, 0.0}, Point2d{1.0, 1.0}),
        1e-12);
    assert(arc.DebugString().find("ArcSegment2d{center=") == 0);
    assert(arc.Kind() == geometry::SegmentKind2::Arc);

    std::unique_ptr<Segment2d> lineSegment = line.Clone();
    std::unique_ptr<Segment2d> arcSegment = arc.Clone();
    assert(lineSegment->Kind() == geometry::SegmentKind2::Line);
    assert(arcSegment->Kind() == geometry::SegmentKind2::Arc);
    GEOMETRY_TEST_ASSERT_NEAR(lineSegment->Length(), 5.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(arcSegment->Length(), std::numbers::pi_v<double> * 0.5, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(lineSegment->StartPoint(), a, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arcSegment->EndPoint(), arcEnd, 1e-12);

    const auto projection = ProjectPointToSegment(Point2d{3.0, 1.0}, a, b);
    const geometry::sdk::SegmentProjection2d expectedProjection{
        Point2d{1.56, 2.08},
        0.52,
        3.24,
        true};
    GEOMETRY_TEST_ASSERT_PROJECTION_NEAR(
        projection,
        expectedProjection,
        1e-12);

    const Box2d boxA = Box2d::FromMinMax(Point2d{0.0, 0.0}, Point2d{2.0, 2.0});
    const Box2d boxB = Box2d::FromMinMax(Point2d{1.0, 1.0}, Point2d{3.0, 3.0});
    const Box2d boxC = Box2d::FromMinMax(Point2d{3.1, 3.1}, Point2d{4.0, 4.0});
    const Box2d invalidBox = Box2d::FromMinMax(Point2d{2.0, 2.0}, Point2d{1.0, 1.0});
    const Box2d expectedBox = Box2d::FromMinMax(Point2d{0.0, 0.0}, Point2d{2.0, 2.0});
    const Point2d expectedCenter{1.0, 1.0};

    assert(boxA.IsValid());
    GEOMETRY_TEST_ASSERT_BOX_NEAR(boxA, expectedBox, 1e-12);
    assert(!invalidBox.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(boxA.Width(), 2.0, 1e-12);
    GEOMETRY_TEST_ASSERT_NEAR(boxA.Height(), 2.0, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(boxA.Center(), expectedCenter, 1e-12);
    assert(Contains(boxA, Point2d{1.0, 1.0}));
    assert(Intersects(boxA, boxB));
    assert(!Intersects(boxA, boxC));
    assert(!Contains(invalidBox, Point2d{1.0, 1.0}));

    const Polyline2d openPath(
        {Point2d{0.0, 0.0}, Point2d{3.0, 0.0}, Point2d{3.0, 4.0}},
        PolylineClosure::Open);
    assert(openPath.IsValid());
    assert(!openPath.IsClosed());
    assert(openPath.PointCount() == 3);
    assert(openPath.SegmentCount() == 2);
    const Point2d openMidPoint{3.0, 0.0};
    GEOMETRY_TEST_ASSERT_POINT_NEAR(openPath.PointAt(1), openMidPoint, 1e-12);
    GEOMETRY_TEST_ASSERT_BOX_NEAR(
        openPath.Bounds(),
        Box2d::FromMinMax(Point2d{0.0, 0.0}, Point2d{3.0, 4.0}),
        1e-12);
    assert(openPath.DebugString().find("Polyline2d{closure=Open") == 0);

    const Polyline2d outerRing(
        {Point2d{0.0, 0.0}, Point2d{4.0, 0.0}, Point2d{4.0, 4.0}, Point2d{0.0, 4.0}},
        PolylineClosure::Closed);
    const Polyline2d holeRing(
        {Point2d{1.0, 1.0}, Point2d{1.0, 3.0}, Point2d{3.0, 3.0}, Point2d{3.0, 1.0}},
        PolylineClosure::Closed);
    const Polygon2d polygon(outerRing, {holeRing});
    assert(polygon.IsValid());
    assert(polygon.HoleCount() == 1);
    assert(polygon.PointCount() == 8);
    assert(polygon.SegmentCount() == 8);
    GEOMETRY_TEST_ASSERT_POLYLINE_NEAR(polygon.OuterRing(), outerRing, 1e-12);
    GEOMETRY_TEST_ASSERT_POLYLINE_NEAR(polygon.HoleAt(0), holeRing, 1e-12);
    GEOMETRY_TEST_ASSERT_BOX_NEAR(
        polygon.Bounds(),
        Box2d::FromMinMax(Point2d{0.0, 0.0}, Point2d{4.0, 4.0}),
        1e-12);
    assert(polygon.DebugString().find("Polygon2d{holeCount=1") == 0);

    return 0;
}
