#include <gtest/gtest.h>
#include <cmath>
#include <concepts>

#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Support/Geometry2d/Normalize2.h"
#include "Support/GeometryTestSupport.h"
#include "Types/Geometry2d/SCBox2.h"

using Geometry::SCArcDirection;
using Geometry::SCArcSegment2d;
using Geometry::SCBox2d;
using Geometry::SCBox2i;
using Geometry::Cross;
using Geometry::Dot;
using Geometry::IsEqual;
using Geometry::IsZero;
using Geometry::SCLineSegment2d;
using Geometry::SCPoint2d;
using Geometry::SCPoint2i;
using Geometry::ISCSegment2d;
using Geometry::SCSegmentKind2;
using Geometry::TryNormalize;
using Geometry::SCVector2d;
using Geometry::SCVector2i;

template <typename T>
concept SupportsPointPlusPoint = requires(T a, T b) { a + b; };

template <typename T>
concept SupportsPointScalarMul = requires(T a) { a * 2; };

template <typename T>
concept SupportsPointNegate = requires(T a) { -a; };

namespace
{
    constexpr double kPi = 3.141592653589793238462643383279502884;
}

TEST(PointVectorTest, CoversCurrentCapabilities)
{
    static_assert(std::is_abstract_v<ISCSegment2d>);
    static_assert(!std::same_as<SCPoint2d, SCVector2d>);
    static_assert(!SupportsPointPlusPoint<SCPoint2d>);
    static_assert(!SupportsPointScalarMul<SCPoint2d>);
    static_assert(!SupportsPointNegate<SCPoint2d>);

    constexpr SCPoint2i pointA(1, 2);
    constexpr SCPoint2i pointB(4, 6);
    constexpr SCVector2i displacement = pointB - pointA;
    static_assert(pointA.IsValid());
    static_assert(displacement == SCVector2i(3, 4));
    static_assert(pointA + displacement == pointB);
    static_assert(pointB - displacement == pointA);

    constexpr SCVector2i vectorA(3, 4);
    constexpr SCVector2i vectorB(-1, 2);
    static_assert(vectorA + vectorB == SCVector2i(2, 6));
    static_assert(vectorA - vectorB == SCVector2i(4, 2));
    static_assert(-vectorB == SCVector2i(1, -2));
    static_assert(vectorA * 2 == SCVector2i(6, 8));
    static_assert(2 * vectorA == SCVector2i(6, 8));
    static_assert(vectorA / 2 == SCVector2i(1, 2));
    static_assert(vectorA.LengthSquared() == 25.0);
    static_assert(vectorA.IsValid());
    ASSERT_LT(std::abs(vectorA.Length() - 5.0), 1e-12);

    static_assert(Dot(vectorA, vectorB) == 5);
    static_assert(Cross(vectorA, vectorB) == 10);
    static_assert(IsZero(0));
    static_assert(!IsZero(1));
    static_assert(IsZero(SCVector2i{}));
    static_assert(!IsZero(vectorA));
    static_assert(IsEqual(SCPoint2d(1.0, 2.0), SCPoint2d(1.0 + 1e-10, 2.0 - 1e-10)));
    GEOMETRY_TEST_ASSERT_POINT_NEAR(SCPoint2d(1.0, 2.0), SCPoint2d(1.0 + 1e-10, 2.0 - 1e-10), 1e-9);
    GEOMETRY_TEST_ASSERT_VECTOR_NEAR(SCVector2d(3.0, 4.0), SCVector2d(3.0 + 1e-10, 4.0 - 1e-10), 1e-9);

    SCVector2d normalized;
    const bool normalizedOk = TryNormalize(SCVector2d(3.0, 4.0), normalized);
    ASSERT_TRUE(normalizedOk);
    ASSERT_LT(std::abs(normalized.x - 0.6), 1e-12);
    ASSERT_LT(std::abs(normalized.y - 0.8), 1e-12);
    ASSERT_LT(std::abs(normalized.Length() - 1.0), 1e-12);

    SCVector2d zeroNormalized(7.0, 9.0);
    const bool zeroOk = TryNormalize(SCVector2d(1e-12, 0.0), zeroNormalized);
    ASSERT_FALSE(zeroOk);
    ASSERT_EQ(zeroNormalized, SCVector2d(7.0, 9.0));

    SCBox2i emptyBox;
    ASSERT_FALSE(emptyBox.IsValid());

    emptyBox.ExpandToInclude(SCPoint2i(2, 3));
    ASSERT_TRUE(emptyBox.IsValid());
    ASSERT_EQ(emptyBox.MinPoint(), SCPoint2i(2, 3));
    ASSERT_EQ(emptyBox.MaxPoint(), SCPoint2i(2, 3));
    ASSERT_EQ(emptyBox.Width(), 0.0);
    ASSERT_EQ(emptyBox.Height(), 0.0);

    const SCBox2d boxA(SCPoint2d(1.0, 2.0), SCPoint2d(4.0, 6.0));
    ASSERT_TRUE(boxA.IsValid());
    ASSERT_LT(std::abs(boxA.Width() - 3.0), 1e-12);
    ASSERT_LT(std::abs(boxA.Height() - 4.0), 1e-12);
    ASSERT_LT(std::abs(boxA.Area() - 12.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(boxA.Center(), SCPoint2d(2.5, 4.0), 1e-12);

    const SCLineSegment2d line(SCPoint2d(1.0, 2.0), SCPoint2d(4.0, 6.0));
    ASSERT_EQ(line.Kind(), SCSegmentKind2::Line);
    ASSERT_TRUE(line.IsValid());
    ASSERT_LT(std::abs(line.Length() - 5.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(line.PointAt(0.5), SCPoint2d(2.5, 4.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(line.PointAtLength(2.5), SCPoint2d(2.5, 4.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(line.PointAtLength(-2.5, true), SCPoint2d(1.0, 2.0), 1e-12);

    const SCBox2d lineBox = line.Bounds();
    ASSERT_TRUE(lineBox.IsValid());
    GEOMETRY_TEST_ASSERT_POINT_NEAR(lineBox.MinPoint(), SCPoint2d(1.0, 2.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(lineBox.MaxPoint(), SCPoint2d(4.0, 6.0), 1e-12);

    const ISCSegment2d& lineAsSegment = line;
    ASSERT_EQ(lineAsSegment.Kind(), SCSegmentKind2::Line);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(lineAsSegment.PointAt(0.5), SCPoint2d(2.5, 4.0), 1e-12);

    const SCArcSegment2d arc(SCPoint2d(0.0, 0.0), 1.0, 0.0, kPi / 2.0, SCArcDirection::CounterClockwise);
    ASSERT_EQ(arc.Kind(), SCSegmentKind2::Arc);
    ASSERT_TRUE(arc.IsValid());
    ASSERT_LT(std::abs(arc.Length() - (kPi / 2.0)), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arc.StartPoint(), SCPoint2d(1.0, 0.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arc.EndPoint(), SCPoint2d(0.0, 1.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arc.PointAt(0.5), SCPoint2d(std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0), 1e-12);

    const SCBox2d arcBox = arc.Bounds();
    ASSERT_TRUE(arcBox.IsValid());
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arcBox.MinPoint(), SCPoint2d(0.0, 0.0), 1e-12);
    GEOMETRY_TEST_ASSERT_POINT_NEAR(arcBox.MaxPoint(), SCPoint2d(1.0, 1.0), 1e-12);
}
