#include <gtest/gtest.h>
#include <cmath>
#include <concepts>

#include "Support/Geometry2d/Normalize2.h"
#include "Support/Epsilon.h"

using Geometry::Cross;
using Geometry::Dot;
using Geometry::IsEqual;
using Geometry::IsZero;
using Geometry::SCPoint2d;
using Geometry::SCPoint2i;
using Geometry::TryNormalize;
using Geometry::SCVector2d;
using Geometry::SCVector2i;

TEST(Vector2Test, Normalized)
{
    const SCVector2d normalized = SCVector2d{3.0, 4.0}.Normalized();
    EXPECT_NEAR(normalized.x, 0.6, 1e-12);
    EXPECT_NEAR(normalized.y, 0.8, 1e-12);
    const SCVector2d zeroNormalized = SCVector2d{0.0, 0.0}.Normalized();
    EXPECT_DOUBLE_EQ(zeroNormalized.x, 0.0);
    EXPECT_DOUBLE_EQ(zeroNormalized.y, 0.0);
    SCVector2d tryNormalized{};
    EXPECT_TRUE((SCVector2d{3.0, 4.0}.TryNormalized(tryNormalized)));
    EXPECT_TRUE(tryNormalized.AlmostEquals(SCVector2d{0.6, 0.8}, 1e-12));
    EXPECT_FALSE(SCVector2d{}.TryNormalized(tryNormalized));
}

TEST(Vector2Test, LengthAndNormalizationRemainStableAtExtremeScales)
{
    const SCVector2d tiny{1e-200, 0.0};
    EXPECT_NEAR(tiny.Length(), 1e-200, 1e-210);
    SCVector2d normalized;
    ASSERT_TRUE(tiny.TryNormalized(normalized, 1e-210));
    EXPECT_EQ(normalized, (SCVector2d{1.0, 0.0}));

    const SCVector2d large{1e200, 0.0};
    EXPECT_NEAR(large.Length(), 1e200, 1e190);
    ASSERT_TRUE(large.TryNormalized(normalized));
    EXPECT_EQ(normalized, (SCVector2d{1.0, 0.0}));
}

template <typename T>
concept SupportsPointPlusPoint = requires(T a, T b) { a + b; };

template <typename T>
concept SupportsPointScalarMul = requires(T a) { a * 2; };

template <typename T>
concept SupportsPointNegate = requires(T a) { -a; };

TEST(PointVectorTest, CoversPointAndVectorContracts)
{
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
    EXPECT_TRUE(SCPoint2d(1.0, 2.0).AlmostEquals(SCPoint2d(1.0 + 1e-10, 2.0 - 1e-10), 1e-9));
    EXPECT_TRUE(SCVector2d(3.0, 4.0).AlmostEquals(SCVector2d(3.0 + 1e-10, 4.0 - 1e-10), 1e-9));

    SCVector2d normalized;
    ASSERT_TRUE(SCVector2d(3.0, 4.0).TryNormalized(normalized));
    ASSERT_LT(std::abs(normalized.x - 0.6), 1e-12);
    ASSERT_LT(std::abs(normalized.y - 0.8), 1e-12);
    ASSERT_LT(std::abs(normalized.Length() - 1.0), 1e-12);

    EXPECT_FALSE(SCVector2d(1e-12, 0.0).TryNormalized(normalized));
}
