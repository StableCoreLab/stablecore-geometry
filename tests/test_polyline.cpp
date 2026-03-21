#include <cassert>
#include <cmath>
#include <memory>
#include <vector>

#include "types/ArcSegment2.h"
#include "types/LineSegment2.h"
#include "types/Polyline2.h"

using geometry::ArcDirection;
using geometry::ArcSegment2d;
using geometry::Box2d;
using geometry::IsEqual;
using geometry::LineSegment2d;
using geometry::Point2d;
using geometry::Polyline2d;
using geometry::PolylineClosure;

namespace
{
constexpr double kPi = 3.141592653589793238462643383279502884;

void ExpectPointNear(const Point2d& actual, const Point2d& expected, double eps = 1e-12)
{
    assert(IsEqual(actual, expected, eps));
}
}

int main()
{
    Polyline2d emptyPath;
    assert(!emptyPath.IsValid());
    assert(emptyPath.GetSegmentCount() == 0);
    assert(emptyPath.GetVertexCount() == 0);

    auto first = std::make_shared<LineSegment2d>(Point2d(0.0, 0.0), Point2d(3.0, 0.0));
    auto second = std::make_shared<LineSegment2d>(Point2d(3.0, 0.0), Point2d(3.0, 4.0));
    Polyline2d openPath({first, second}, PolylineClosure::Open);

    assert(openPath.IsValid());
    assert(!openPath.IsClosed());
    assert(openPath.GetSegmentCount() == 2);
    assert(openPath.GetVertexCount() == 3);
    ExpectPointNear(openPath.GetVertex(0), Point2d(0.0, 0.0));
    ExpectPointNear(openPath.GetVertex(1), Point2d(3.0, 0.0));
    ExpectPointNear(openPath.GetVertex(2), Point2d(3.0, 4.0));
    ExpectPointNear(openPath.GetStartPoint(), Point2d(0.0, 0.0));
    ExpectPointNear(openPath.GetEndPoint(), Point2d(3.0, 4.0));
    assert(std::abs(openPath.Length() - 7.0) < 1e-12);
    assert(std::abs(openPath.GetLengthAt(0.5) - 3.5) < 1e-12);
    assert(std::abs(openPath.GetParameterAtLength(3.5) - 0.5) < 1e-12);
    ExpectPointNear(openPath.GetPointAt(0.5), Point2d(3.0, 0.5));
    ExpectPointNear(openPath.GetPointAtLength(3.5), Point2d(3.0, 0.5));
    ExpectPointNear(openPath.GetPointAtLength(-2.0, true), Point2d(0.0, 0.0));
    ExpectPointNear(openPath.GetPointAtLength(9.0, true), Point2d(3.0, 4.0));
    ExpectPointNear(openPath.GetPointAtLength(-2.0, false), Point2d(-2.0, 0.0));
    ExpectPointNear(openPath.GetPointAtLength(9.0, false), Point2d(3.0, 6.0));

    const Box2d openBox = openPath.GetBoundingBox();
    assert(openBox.IsValid());
    ExpectPointNear(openBox.GetMinPoint(), Point2d(0.0, 0.0));
    ExpectPointNear(openBox.GetMaxPoint(), Point2d(3.0, 4.0));

    auto broken = std::make_shared<LineSegment2d>(Point2d(10.0, 10.0), Point2d(11.0, 10.0));
    Polyline2d invalidOpen({first, broken}, PolylineClosure::Open);
    assert(!invalidOpen.IsValid());

    auto closeA = std::make_shared<LineSegment2d>(Point2d(0.0, 0.0), Point2d(1.0, 0.0));
    auto closeB = std::make_shared<LineSegment2d>(Point2d(1.0, 0.0), Point2d(0.0, 1.0));
    auto closeC = std::make_shared<LineSegment2d>(Point2d(0.0, 1.0), Point2d(0.0, 0.0));
    Polyline2d closedPath({closeA, closeB, closeC}, PolylineClosure::Closed);

    assert(closedPath.IsValid());
    assert(closedPath.IsClosed());
    assert(closedPath.GetVertexCount() == 3);
    ExpectPointNear(closedPath.GetStartPoint(), Point2d(0.0, 0.0));
    ExpectPointNear(closedPath.GetEndPoint(), Point2d(0.0, 0.0));

    auto line = std::make_shared<LineSegment2d>(Point2d(0.0, 0.0), Point2d(1.0, 0.0));
    auto arc = std::make_shared<ArcSegment2d>(
        Point2d(1.0, 1.0),
        1.0,
        -kPi / 2.0,
        0.0,
        ArcDirection::CounterClockwise);
    Polyline2d mixedPath({line, arc}, PolylineClosure::Open);

    assert(mixedPath.IsValid());
    assert(std::abs(mixedPath.Length() - (1.0 + kPi / 2.0)) < 1e-12);
    const Box2d mixedBox = mixedPath.GetBoundingBox();
    assert(mixedBox.IsValid());
    ExpectPointNear(mixedBox.GetMinPoint(), Point2d(0.0, 0.0));
    ExpectPointNear(mixedBox.GetMaxPoint(), Point2d(2.0, 1.0));

    return 0;
}
