#include <cassert>
#include <vector>

#include "sdk/GeometryBoolean.h"
#include "sdk/GeometryRelation.h"
#include "sdk/GeometryShapeOps.h"
#include "sdk/GeometryValidation.h"
#include "support/GeometryTestSupport.h"

using geometry::sdk::LineSegment2d;
using geometry::sdk::LocatePoint;
using geometry::sdk::Point2d;
using geometry::sdk::PointContainment2d;
using geometry::sdk::Polygon2d;
using geometry::sdk::Polyline2d;
using geometry::sdk::PolylineClosure;

namespace
{
double TotalArea(const geometry::sdk::MultiPolygon2d& polygons)
{
    double total = 0.0;
    for (std::size_t i = 0; i < polygons.Count(); ++i)
    {
        total += geometry::sdk::Area(polygons[i]);
    }
    return total;
}
}

int main()
{
    const LineSegment2d horizontal(Point2d{0.0, 0.0}, Point2d{4.0, 0.0});
    assert(LocatePoint(Point2d{2.0, 0.0}, horizontal) == PointContainment2d::OnBoundary);
    assert(LocatePoint(Point2d{2.0, 1.0}, horizontal) == PointContainment2d::Outside);

    const Polyline2d squareRing(
        {Point2d{0.0, 0.0}, Point2d{4.0, 0.0}, Point2d{4.0, 4.0}, Point2d{0.0, 4.0}},
        PolylineClosure::Closed);
    const Polygon2d square(squareRing);
    assert(LocatePoint(Point2d{2.0, 2.0}, square) == PointContainment2d::Inside);
    assert(LocatePoint(Point2d{5.0, 2.0}, square) == PointContainment2d::Outside);
    assert(geometry::sdk::Validate(square).valid);

    const Polyline2d clipRing(
        {Point2d{2.0, 0.0}, Point2d{6.0, 0.0}, Point2d{6.0, 4.0}, Point2d{2.0, 4.0}},
        PolylineClosure::Closed);
    const Polygon2d clip(clipRing);
    const auto intersection = geometry::sdk::Intersect(square, clip);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(intersection), 8.0, 1e-9);

    const auto united = geometry::sdk::Union(square, clip);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(united), 24.0, 1e-9);

    const Polyline2d innerRing(
        {Point2d{1.0, 1.0}, Point2d{3.0, 1.0}, Point2d{3.0, 3.0}, Point2d{1.0, 3.0}},
        PolylineClosure::Closed);
    const Polygon2d inner(innerRing);
    const auto difference = geometry::sdk::Difference(square, inner);
    GEOMETRY_TEST_ASSERT_NEAR(TotalArea(difference), 12.0, 1e-9);

    return 0;
}
