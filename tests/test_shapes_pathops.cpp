#include <cassert>

#include "sdk/Geometry.h"
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

int main()
{
    const Circle2d circle(Point2d{1.0, 2.0}, 3.0);
    assert(circle.IsValid());
    GEOMETRY_TEST_ASSERT_NEAR(circle.Area(), 28.274333882308138, 1e-9);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(circle.Bounds().GetMinPoint(), (Point2d{-2.0, -1.0}), 1e-12);
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

    const MultiPolyline2d lines{
        Polyline2d(
            {Point2d{0.0, 0.0}, Point2d{2.0, 0.0}, Point2d{2.0, 2.0}, Point2d{0.0, 2.0}},
            PolylineClosure::Closed),
        Polyline2d(
            {Point2d{5.0, 5.0}, Point2d{6.0, 5.0}, Point2d{6.0, 6.0}, Point2d{5.0, 6.0}},
            PolylineClosure::Closed)};
    const auto multi = BuildMultiPolygonByLines(lines);
    assert(multi.Count() == 2);

    return 0;
}
