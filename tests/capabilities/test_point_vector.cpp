#include <gtest/gtest.h>
#include <cassert>
#include <concepts>
#include <cmath>

#include "algorithm/Normalize2.h"
#include "types/ArcSegment2.h"
#include "types/Box2.h"
#include "types/LineSegment2.h"
#include "support/GTestCompat.h"
#include "support/GeometryTestSupport.h"

using geometry::ArcDirection;
using geometry::ArcSegment2d;
using geometry::Box2d;
using geometry::Box2i;
using geometry::Cross;
using geometry::Dot;
using geometry::IsEqual;
using geometry::IsZero;
using geometry::LineSegment2d;
using geometry::Point2d;
using geometry::Point2i;
using geometry::Segment2d;
using geometry::SegmentKind2;
using geometry::TryNormalize;
using geometry::Vector2d;
using geometry::Vector2i;

template <typename T>
concept SupportsPointPlusPoint = requires(T a, T b) {
    a + b;
};

template <typename T>
concept SupportsPointScalarMul = requires(T a) {
    a * 2;
};

template <typename T>
concept SupportsPointNegate = requires(T a) {
    -a;
};

namespace
{
constexpr double kPi = 3.141592653589793238462643383279502884;
}

TEST(PointVectorTest, CoversCurrentCapabilities)
{
    static_assert(std::is_abstract_v<Segment2d>);
    static_assert(!std::same_as<Point2d, Vector2d>);
    static_assert(!SupportsPointPlusPoint<Point2d>);
    static_assert(!SupportsPointScalarMul<Point2d>);
    static_assert(!SupportsPointNegate<Point2d>);

    constexpr Point2i pointA(1, 2);
    constexpr Point2i pointB(4, 6);
    constexpr Vector2i displacement = pointB - pointA;
    static_assert(pointA.IsValid());
    static_assert(displacement == Vector2i(3, 4));
    static_assert(pointA + displacement == pointB);
    static_assert(pointB - displacement == pointA);

    constexpr Vector2i vectorA(3, 4);
    constexpr Vector2i vectorB(-1, 2);
    static_assert(vectorA + vectorB == Vector2i(2, 6));
    static_assert(vectorA - vectorB == Vector2i(4, 2));
    static_assert(-vectorB == Vector2i(1, -2));
    static_assert(vectorA * 2 == Vector2i(6, 8));
    static_assert(2 * vectorA == Vector2i(6, 8));
    static_assert(vectorA / 2 == Vector2i(1, 2));
    static_assert(vectorA.LengthSquared() == 25.0);
    static_assert(vectorA.IsValid());
    assert(std::abs(vectorA.Length() - 5.0) < 1e-12);

    static_assert(Dot(vectorA, vectorB) == 5);
    static_assert(Cross(vectorA, vectorB) == 10);
    static_assert(IsZero(0));
    static_assert(!IsZero(1));
    static_assert(IsZero(Vector2i{}));
    static_assert(!IsZero(vectorA));
    static_assert(IsEqual(Point2d(1.0, 2.0), Point2d(1.0 + 1e-10, 2.0 - 1e-10)));
    GEOMETRY_TEST_ASSERT_POINT_NEAR(
        Point2d(1.0, 2.0),
        Point2d(1.0 + 1e-10, 2.0 - 1e-10),
        1e-9);
    GEOMETRY_TEST_ASSERT_VECTOR_NEAR(
        Vector2d(3.0, 4.0),
        Vector2d(3.0 + 1e-10, 4.0 - 1e-10),
        1e-9);

    Vector2d normalized;
    const bool normalizedOk = TryNormalize(Vector2d(3.0, 4.0), normalized);
    assert(normalizedOk);
    assert(std::abs(normalized.x - 0.6) < 1e-12);
    assert(std::abs(normalized.y - 0.8) < 1e-12);
    assert(std::abs(normalized.Length() - 1.0) < 1e-12);

    Vector2d zeroNormalized(7.0, 9.0);
    const bool zeroOk = TryNormalize(Vector2d(1e-12, 0.0), zeroNormalized);
    assert(!zeroOk);
    assert(zeroNormalized == Vector2d(7.0, 9.0));

    Box2i emptyBox;
    assert(!emptyBox.IsValid());

    emptyBox.ExpandToInclude(Point2i(2, 3));
    assert(emptyBox.IsValid());
    assert(emptyBox.MinPoint() == Point2i(2, 3));
    assert(emptyBox.MaxPoint() == Point2i(2, 3));
    assert(emptyBox.Width() == 0.0);
    assert(emptyBox.Height() == 0.0);

    const Box2d boxA(Point2d(1.0, 2.0), Point2d(4.0, 6.0));
    assert(boxA.IsValid());
    assert(std::abs(boxA.Width() - 3.0) < 1e-12);
    assert(std::abs(boxA.Height() - 4.0) < 1e-12);
    assert(std::abs(boxA.Area() - 12.0) < 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(boxA.Center(), Point2d(2.5, 4.0), 1e-12);

    const LineSegment2d line(Point2d(1.0, 2.0), Point2d(4.0, 6.0));
    assert(line.Kind() == SegmentKind2::Line);
    assert(line.IsValid());
    assert(std::abs(line.Length() - 5.0) < 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(line.PointAt(0.5), Point2d(2.5, 4.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(line.PointAtLength(2.5), Point2d(2.5, 4.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(line.PointAtLength(-2.5, true), Point2d(1.0, 2.0), 1e-12);

    const Box2d lineBox = line.Bounds();
    assert(lineBox.IsValid());
    GEOMETRY_TEST_ASSERT_POINT_NEAR(lineBox.MinPoint(), Point2d(1.0, 2.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(lineBox.MaxPoint(), Point2d(4.0, 6.0), 1e-12);

    const Segment2d& lineAsSegment = line;
    assert(lineAsSegment.Kind() == SegmentKind2::Line);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(lineAsSegment.PointAt(0.5), Point2d(2.5, 4.0), 1e-12);

    const ArcSegment2d arc(
        Point2d(0.0, 0.0),
        1.0,
        0.0,
        kPi / 2.0,
        ArcDirection::CounterClockwise);
    assert(arc.Kind() == SegmentKind2::Arc);
    assert(arc.IsValid());
    assert(std::abs(arc.Length() - (kPi / 2.0)) < 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arc.StartPoint(), Point2d(1.0, 0.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arc.EndPoint(), Point2d(0.0, 1.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(
        arc.PointAt(0.5),
        Point2d(std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0),
        1e-12);

    const Box2d arcBox = arc.Bounds();
    assert(arcBox.IsValid());
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arcBox.MinPoint(), Point2d(0.0, 0.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arcBox.MaxPoint(), Point2d(1.0, 1.0), 1e-12);
}



