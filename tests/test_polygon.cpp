#include <cassert>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "types/ArcSegment2.h"
#include "types/LineSegment2.h"
#include "types/Polygon2.h"

using geometry::Box2d;
using geometry::ArcDirection;
using geometry::ArcSegment2d;
using geometry::IsEqual;
using geometry::LineSegment2d;
using geometry::Point2d;
using geometry::Polygon2d;
using geometry::Polyline2d;
using geometry::PolylineClosure;

namespace
{
void ExpectPointNear(const Point2d& actual, const Point2d& expected, double eps = 1e-12)
{
    assert(IsEqual(actual, expected, eps));
}

Polyline2d MakeClosedRing(const std::vector<std::pair<Point2d, Point2d>>& edges)
{
    std::vector<std::shared_ptr<geometry::Segment2d>> segments;
    segments.reserve(edges.size());
    for (const auto& edge : edges)
    {
        segments.push_back(std::make_shared<LineSegment2d>(edge.first, edge.second));
    }
    return Polyline2d(std::move(segments), PolylineClosure::Closed);
}
}

int main()
{
    Polyline2d outerRing = MakeClosedRing({
        {Point2d(0.0, 0.0), Point2d(4.0, 0.0)},
        {Point2d(4.0, 0.0), Point2d(4.0, 4.0)},
        {Point2d(4.0, 4.0), Point2d(0.0, 4.0)},
        {Point2d(0.0, 4.0), Point2d(0.0, 0.0)},
    });

    Polyline2d holeRing = MakeClosedRing({
        {Point2d(1.0, 1.0), Point2d(1.0, 3.0)},
        {Point2d(1.0, 3.0), Point2d(3.0, 3.0)},
        {Point2d(3.0, 3.0), Point2d(3.0, 1.0)},
        {Point2d(3.0, 1.0), Point2d(1.0, 1.0)},
    });

    Polygon2d polygon(outerRing, {holeRing});
    assert(polygon.IsValid());
    assert(polygon.GetOuterRing().IsClosed());
    assert(polygon.GetHoleCount() == 1);
    assert(polygon.GetHole(0).IsClosed());

    assert(std::abs(polygon.Area() - 12.0) < 1e-12);
    assert(std::abs(polygon.Perimeter() - 24.0) < 1e-12);
    ExpectPointNear(polygon.Centroid(), Point2d(2.0, 2.0));

    const Box2d box = polygon.GetBoundingBox();
    assert(box.IsValid());
    ExpectPointNear(box.GetMinPoint(), Point2d(0.0, 0.0));
    ExpectPointNear(box.GetMaxPoint(), Point2d(4.0, 4.0));

    Polyline2d badHole = MakeClosedRing({
        {Point2d(1.0, 1.0), Point2d(3.0, 1.0)},
        {Point2d(3.0, 1.0), Point2d(3.0, 3.0)},
        {Point2d(3.0, 3.0), Point2d(1.0, 3.0)},
        {Point2d(1.0, 3.0), Point2d(1.0, 1.0)},
    });
    Polygon2d invalidPolygon(outerRing, {badHole});
    assert(!invalidPolygon.IsValid());

    std::vector<std::shared_ptr<geometry::Segment2d>> arcSegments;
    arcSegments.push_back(std::make_shared<LineSegment2d>(Point2d(0.0, 0.0), Point2d(1.0, 0.0)));
    arcSegments.push_back(std::make_shared<ArcSegment2d>(
        Point2d(0.0, 0.0),
        1.0,
        0.0,
        std::acos(-1.0) * 0.5,
        ArcDirection::CounterClockwise));
    arcSegments.push_back(std::make_shared<LineSegment2d>(Point2d(0.0, 1.0), Point2d(0.0, 0.0)));

    Polygon2d quarterDisk(Polyline2d(std::move(arcSegments), PolylineClosure::Closed));
    assert(quarterDisk.IsValid());
    assert(std::abs(quarterDisk.Area() - (std::acos(-1.0) / 4.0)) < 1e-12);
    assert(std::abs(quarterDisk.Perimeter() - (2.0 + std::acos(-1.0) * 0.5)) < 1e-12);
    ExpectPointNear(
        quarterDisk.Centroid(),
        Point2d(4.0 / (3.0 * std::acos(-1.0)), 4.0 / (3.0 * std::acos(-1.0))),
        1e-12);

    return 0;
}
