#include <gtest/gtest.h>
#include <cassert>
#include <cmath>
#include <concepts>
#include <type_traits>

#include "types/ArcSegment2.h"
#include "types/LineSegment2.h"
#include "support/GTestCompat.h"
#include "support/GeometryTestSupport.h"

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
}

TEST(SegmentTest, CoversCurrentCapabilities)
{
    static_assert(std::is_abstract_v<Segment2d>);
    static_assert(!std::is_abstract_v<LineSegment2d>);
    static_assert(!std::is_abstract_v<ArcSegment2d>);

    const Point2d start(1.0, 2.0);
    const Point2d end(4.0, 6.0);
    const LineSegment2d line(start, end);

    assert(line.Kind() == SegmentKind2::Line);
    assert(line.IsValid());
    GEOMETRY_TEST_ASSERT_POINT_NEAR(line.StartPoint(), start, 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(line.EndPoint(), end, 1e-12);
    assert(std::abs(line.Length() - 5.0) < 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(line.PointAt(0.5), Point2d(2.5, 4.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(line.PointAtLength(2.5), Point2d(2.5, 4.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(line.PointAtLength(-2.5, false), Point2d(-0.5, 0.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(line.PointAtLength(-2.5, true), start, 1e-12);

    const Segment2d& lineAsBase = line;
    assert(lineAsBase.Kind() == SegmentKind2::Line);
    assert(std::abs(lineAsBase.Length() - 5.0) < 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(lineAsBase.PointAt(0.5), Point2d(2.5, 4.0), 1e-12);

    const Box2d lineBox = line.Bounds();
    assert(lineBox.IsValid());
    GEOMETRY_TEST_ASSERT_POINT_NEAR(lineBox.MinPoint(), Point2d(1.0, 2.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(lineBox.MaxPoint(), Point2d(4.0, 6.0), 1e-12);

    const LineSegment2d degenerateLine(start, start);
    assert(!degenerateLine.IsValid());

    const ArcSegment2d quarterArc(
        Point2d(0.0, 0.0),
        1.0,
        0.0,
        kPi / 2.0,
        ArcDirection::CounterClockwise);

    assert(quarterArc.Kind() == SegmentKind2::Arc);
    assert(quarterArc.IsValid());
    assert(std::abs(quarterArc.Length() - (kPi / 2.0)) < 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(quarterArc.StartPoint(), Point2d(1.0, 0.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(quarterArc.EndPoint(), Point2d(0.0, 1.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(
        quarterArc.PointAt(0.5),
        Point2d(std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0),
        1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(
        quarterArc.PointAtLength(quarterArc.Length() * 0.5),
        Point2d(std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0),
        1e-12);

    const Box2d arcBox = quarterArc.Bounds();
    assert(arcBox.IsValid());
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arcBox.MinPoint(), Point2d(0.0, 0.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arcBox.MaxPoint(), Point2d(1.0, 1.0), 1e-12);

    const Segment2d& arcAsBase = quarterArc;
    assert(arcAsBase.Kind() == SegmentKind2::Arc);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arcAsBase.PointAt(0.0), Point2d(1.0, 0.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arcAsBase.PointAt(1.0), Point2d(0.0, 1.0), 1e-12);

    const ArcSegment2d clockwiseArc(
        Point2d(0.0, 0.0),
        1.0,
        kPi / 2.0,
        0.0,
        ArcDirection::Clockwise);

    assert(clockwiseArc.IsValid());
    assert(std::abs(clockwiseArc.Length() - (kPi / 2.0)) < 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(clockwiseArc.StartPoint(), Point2d(0.0, 1.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(clockwiseArc.EndPoint(), Point2d(1.0, 0.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(
        clockwiseArc.PointAt(0.5),
        Point2d(std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0),
        1e-12);

    const ArcSegment2d invalidArc(
        Point2d(0.0, 0.0),
        1.0,
        0.0,
        0.0,
        ArcDirection::CounterClockwise);

    assert(!invalidArc.IsValid());
}



