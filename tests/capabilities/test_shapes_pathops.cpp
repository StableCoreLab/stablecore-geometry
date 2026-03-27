#include <gtest/gtest.h>
#include <cassert>

#include "sdk/Geometry.h"
#include "support/GTestCompat.h"
#include "support/GeometryTestSupport.h"

using geometry::sdk::BuildMultiPolygonByLines;
using geometry::sdk::Circle2d;
using geometry::sdk::CutPolygon;
using geometry::sdk::Ellipse2d;
using geometry::sdk::LineSegment2d;
using geometry::sdk::MultiPolyline2d;
using geometry::sdk::Point2d;
using geometry::sdk::Polygon2d;
using geometry::sdk::Polyline2d;
using geometry::sdk::PolylineClosure;
using geometry::sdk::Rectangle2d;
using geometry::sdk::SubPolyline;

TEST(ShapesPathopsTest, CoversCurrentCapabilities)
{
    const Circle2d circle(Point2d{1.0, 2.0}, 3.0);
    assert(circle.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(circle.Area(), 28.274333882308138, 1e-9);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(circle.Bounds().MinPoint(), (Point2d{-2.0, -1.0}), 1e-12);
    assert(circle.ToPolygon(16).IsValid());

    const Rectangle2d rectangle(Point2d{0.0, 0.0}, 4.0, 2.0, 0.0);
    assert(rectangle.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(rectangle.Area(), 8.0, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(rectangle.ToPolygon().OuterRing().PointAt(0), (Point2d{-2.0, -1.0}), 1e-12);

    const Ellipse2d ellipse(Point2d{0.0, 0.0}, 3.0, 2.0, 0.0);
    assert(ellipse.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(ellipse.Area(), 18.84955592153876, 1e-9);
    assert(ellipse.ToPolygon(32).IsValid());

    const Polyline2d path(
        {Point2d{0.0, 0.0}, Point2d{3.0, 0.0}, Point2d{3.0, 4.0}},
        PolylineClosure::Open);
    const Polyline2d sub = SubPolyline(path, 2.0, 5.0);
    assert(sub.IsValid());
    GEOMETRY_TEST_ASSERT_POINT_NEAR(sub.PointAt(0), (Point2d{2.0, 0.0}), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(sub.PointAt(sub.PointCount() - 1), (Point2d{3.0, 2.0}), 1e-12);

    const Polygon2d square(
        Polyline2d(
            {Point2d{0.0, 0.0}, Point2d{4.0, 0.0}, Point2d{4.0, 4.0}, Point2d{0.0, 4.0}},
            PolylineClosure::Closed));
    const auto cut = CutPolygon(square, LineSegment2d(Point2d{2.0, -1.0}, Point2d{2.0, 5.0}));
    assert(cut.success);
    assert(cut.left.Count() == 1);
    assert(cut.right.Count() == 1);
    GEOMETRY_TEST_ASSERT_NEAR(geometry::sdk::Area(cut.left[0]), 8.0, 1e-9);
    GEOMETRY_TEST_ASSERT_NEAR(geometry::sdk::Area(cut.right[0]), 8.0, 1e-9);

    const MultiPolyline2d closedLines{
        Polyline2d(
            {Point2d{0.0, 0.0}, Point2d{2.0, 0.0}, Point2d{2.0, 2.0}, Point2d{0.0, 2.0}},
            PolylineClosure::Closed),
        Polyline2d(
            {Point2d{5.0, 5.0}, Point2d{6.0, 5.0}, Point2d{6.0, 6.0}, Point2d{5.0, 6.0}},
            PolylineClosure::Closed)};
    const auto closedMulti = BuildMultiPolygonByLines(closedLines);
    assert(closedMulti.Count() == 2);

    const MultiPolyline2d openSquareLines{
        Polyline2d({Point2d{0.0, 0.0}, Point2d{4.0, 0.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{4.0, 0.0}, Point2d{4.0, 4.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{4.0, 4.0}, Point2d{0.0, 4.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{0.0, 4.0}, Point2d{0.0, 0.0}}, PolylineClosure::Open)};
    const auto openSquare = BuildMultiPolygonByLines(openSquareLines);
    assert(openSquare.Count() == 1);
    assert(openSquare[0].HoleCount() == 0);
    GEOMETRY_TEST_ASSERT_NEAR(geometry::sdk::Area(openSquare[0]), 16.0, 1e-9);

    const MultiPolyline2d nestedOpenLines{
        Polyline2d({Point2d{0.0, 0.0}, Point2d{6.0, 0.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{6.0, 0.0}, Point2d{6.0, 6.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{6.0, 6.0}, Point2d{0.0, 6.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{0.0, 6.0}, Point2d{0.0, 0.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{2.0, 2.0}, Point2d{4.0, 2.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{4.0, 2.0}, Point2d{4.0, 4.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{4.0, 4.0}, Point2d{2.0, 4.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{2.0, 4.0}, Point2d{2.0, 2.0}}, PolylineClosure::Open)};
    const auto nested = BuildMultiPolygonByLines(nestedOpenLines);
    assert(nested.Count() == 1);
    assert(nested[0].HoleCount() == 1);
    GEOMETRY_TEST_ASSERT_NEAR(geometry::sdk::Area(nested[0]), 32.0, 1e-9);

    const MultiPolyline2d branchedLines{
        Polyline2d({Point2d{0.0, 0.0}, Point2d{4.0, 0.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{4.0, 0.0}, Point2d{4.0, 4.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{4.0, 4.0}, Point2d{0.0, 4.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{0.0, 4.0}, Point2d{0.0, 0.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{2.0, -1.0}, Point2d{2.0, 2.0}}, PolylineClosure::Open)};
    const auto branched = BuildMultiPolygonByLines(branchedLines);
    assert(branched.Count() == 1);
    GEOMETRY_TEST_ASSERT_NEAR(geometry::sdk::Area(branched[0]), 16.0, 1e-9);

    const MultiPolyline2d dirtyNearClosedLines{
        Polyline2d({Point2d{0.0, 0.0}, Point2d{4.0, 0.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{4.0, 0.0}, Point2d{4.0, 4.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{4.0, 4.0}, Point2d{0.0, 4.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{0.0, 4.0}, Point2d{0.0, 0.12}}, PolylineClosure::Open),
        Polyline2d({Point2d{0.0, 0.0}, Point2d{4.0, 0.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{2.0, 4.0}, Point2d{2.0, 5.0}}, PolylineClosure::Open)};
    const auto dirtyNearClosed = BuildMultiPolygonByLines(dirtyNearClosedLines);
    assert(dirtyNearClosed.Count() == 1);
    GEOMETRY_TEST_ASSERT_NEAR(geometry::sdk::Area(dirtyNearClosed[0]), 16.0, 1e-6);

    const MultiPolyline2d autoExtendLines{
        Polyline2d({Point2d{0.0, 0.0}, Point2d{4.0, 0.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{4.0, 0.0}, Point2d{4.0, 4.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{4.0, 4.0}, Point2d{0.0, 4.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{0.0, 4.0}, Point2d{0.0, 1.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{-0.15, 1.0}, Point2d{-0.15, 0.25}}, PolylineClosure::Open)};
    const auto autoExtended = BuildMultiPolygonByLines(autoExtendLines);
    assert(autoExtended.Count() == 1);
    assert(geometry::sdk::Area(autoExtended[0]) >= 15.5);
    assert(geometry::sdk::Area(autoExtended[0]) <= 16.5);

    const MultiPolyline2d ambiguousFakeLines{
        Polyline2d({Point2d{0.0, 0.0}, Point2d{4.0, 0.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{4.0, 0.0}, Point2d{4.0, 4.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{4.0, 4.0}, Point2d{0.0, 4.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{0.0, 4.0}, Point2d{0.0, 0.25}}, PolylineClosure::Open),
        Polyline2d({Point2d{-0.15, 0.25}, Point2d{-0.15, 0.0}}, PolylineClosure::Open),
        Polyline2d({Point2d{-0.15, 0.0}, Point2d{0.15, 0.0}}, PolylineClosure::Open)};
    const auto ambiguousFake = BuildMultiPolygonByLines(ambiguousFakeLines);
    assert(ambiguousFake.Count() == 1);
    assert(ambiguousFake[0].HoleCount() == 0);
    assert(geometry::sdk::Area(ambiguousFake[0]) >= 15.5);
    assert(geometry::sdk::Area(ambiguousFake[0]) <= 16.5);
}



