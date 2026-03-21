#include <cassert>
#include <cmath>
#include <concepts>
#include <type_traits>

#include "types/ArcSegment2.h"
#include "types/LineSegment2.h"

using geometry::ArcDirection;
using geometry::ArcSegment2d;
using geometry::Box2d;
using geometry::IsEqual;
using geometry::LineSegment2d;
using geometry::Point2d;
using geometry::Segment2d;
using geometry::SegmentKind2;

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
    static_assert(std::is_abstract_v<Segment2d>);
    static_assert(!std::is_abstract_v<LineSegment2d>);
    static_assert(!std::is_abstract_v<ArcSegment2d>);

    const Point2d start(1.0, 2.0);
    const Point2d end(4.0, 6.0);
    const LineSegment2d line(start, end);

    assert(line.GetKind() == SegmentKind2::Line);
    assert(line.IsValid());
    ExpectPointNear(line.GetStartPoint(), start);
    ExpectPointNear(line.GetEndPoint(), end);
    assert(std::abs(line.Length() - 5.0) < 1e-12);
    ExpectPointNear(line.GetPointAt(0.5), Point2d(2.5, 4.0));
    ExpectPointNear(line.GetPointAtLength(2.5), Point2d(2.5, 4.0));
    ExpectPointNear(line.GetPointAtLength(-2.5, false), Point2d(-0.5, 0.0));
    ExpectPointNear(line.GetPointAtLength(-2.5, true), start);

    const Segment2d& lineAsBase = line;
    assert(lineAsBase.GetKind() == SegmentKind2::Line);
    assert(std::abs(lineAsBase.Length() - 5.0) < 1e-12);
    ExpectPointNear(lineAsBase.GetPointAt(0.5), Point2d(2.5, 4.0));

    const Box2d lineBox = line.GetBoundingBox();
    assert(lineBox.IsValid());
    ExpectPointNear(lineBox.GetMinPoint(), Point2d(1.0, 2.0));
    ExpectPointNear(lineBox.GetMaxPoint(), Point2d(4.0, 6.0));

    const LineSegment2d degenerateLine(start, start);
    assert(!degenerateLine.IsValid());

    const ArcSegment2d quarterArc(
        Point2d(0.0, 0.0),
        1.0,
        0.0,
        kPi / 2.0,
        ArcDirection::CounterClockwise);

    assert(quarterArc.GetKind() == SegmentKind2::Arc);
    assert(quarterArc.IsValid());
    assert(std::abs(quarterArc.Length() - (kPi / 2.0)) < 1e-12);
    ExpectPointNear(quarterArc.GetStartPoint(), Point2d(1.0, 0.0));
    ExpectPointNear(quarterArc.GetEndPoint(), Point2d(0.0, 1.0));
    ExpectPointNear(
        quarterArc.GetPointAt(0.5),
        Point2d(std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0),
        1e-12);
    ExpectPointNear(
        quarterArc.GetPointAtLength(quarterArc.Length() * 0.5),
        Point2d(std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0),
        1e-12);

    const Box2d arcBox = quarterArc.GetBoundingBox();
    assert(arcBox.IsValid());
    ExpectPointNear(arcBox.GetMinPoint(), Point2d(0.0, 0.0));
    ExpectPointNear(arcBox.GetMaxPoint(), Point2d(1.0, 1.0));

    const Segment2d& arcAsBase = quarterArc;
    assert(arcAsBase.GetKind() == SegmentKind2::Arc);
    ExpectPointNear(arcAsBase.GetPointAt(0.0), Point2d(1.0, 0.0));
    ExpectPointNear(arcAsBase.GetPointAt(1.0), Point2d(0.0, 1.0));

    const ArcSegment2d clockwiseArc(
        Point2d(0.0, 0.0),
        1.0,
        kPi / 2.0,
        0.0,
        ArcDirection::Clockwise);

    assert(clockwiseArc.IsValid());
    assert(std::abs(clockwiseArc.Length() - (kPi / 2.0)) < 1e-12);
    ExpectPointNear(clockwiseArc.GetStartPoint(), Point2d(0.0, 1.0));
    ExpectPointNear(clockwiseArc.GetEndPoint(), Point2d(1.0, 0.0));
    ExpectPointNear(
        clockwiseArc.GetPointAt(0.5),
        Point2d(std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0),
        1e-12);

    const ArcSegment2d invalidArc(
        Point2d(0.0, 0.0),
        1.0,
        0.0,
        0.0,
        ArcDirection::CounterClockwise);

    assert(!invalidArc.IsValid());

    return 0;
}
