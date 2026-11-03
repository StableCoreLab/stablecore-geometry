#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "Support/Geometry2d/Predicate2.h"

TEST(Predicate2dTest, FloatingPredicatesRejectInvalidTolerance)
{
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (const double eps : {-1.0, infinity, -infinity, nan})
    {
        SCOPED_TRACE(eps);
        EXPECT_FALSE(Geometry::IsZero(0.0, eps));
        EXPECT_FALSE(Geometry::IsZero(0.0f, eps));
        EXPECT_FALSE(Geometry::IsEqual(0.0, 0.0, eps));
        EXPECT_FALSE(Geometry::IsEqual(0.0f, 0.0f, eps));
        EXPECT_FALSE(Geometry::IsEqual(Geometry::SCPoint2d{}, Geometry::SCPoint2d{}, eps));
        EXPECT_FALSE(Geometry::IsEqual(Geometry::SCVector2d{}, Geometry::SCVector2d{}, eps));
        EXPECT_FALSE(Geometry::IsZero(Geometry::SCVector2d{}, eps));
    }
}

TEST(Predicate2dTest, FloatingPredicatesRejectNonFiniteValues)
{
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (const double value : {infinity, -infinity, nan})
    {
        EXPECT_FALSE(Geometry::IsZero(value));
        EXPECT_FALSE(Geometry::IsEqual(value, 0.0));
        EXPECT_FALSE(Geometry::IsEqual(0.0, value));
        EXPECT_FALSE(Geometry::IsEqual(Geometry::SCPoint2d{value, 0.0}, Geometry::SCPoint2d{}));
        EXPECT_FALSE(Geometry::IsEqual(Geometry::SCVector2d{0.0, value}, Geometry::SCVector2d{}));
    }
    // 有限 double 容差窄化到 float 后可能变为无穷，不能使无穷输入通过。
    const double maximum = (std::numeric_limits<double>::max)();
    const float floatInfinity = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(Geometry::IsZero(floatInfinity, maximum));
    EXPECT_FALSE(Geometry::IsZero(-floatInfinity, maximum));
    EXPECT_FALSE(Geometry::IsEqual(floatInfinity, 0.0f, maximum));
    EXPECT_FALSE(Geometry::IsEqual(0.0f, -floatInfinity, maximum));
    EXPECT_TRUE(Geometry::IsZero(1.0f, maximum));
    EXPECT_TRUE(Geometry::IsEqual(1.0f, 2.0f, maximum));
}

TEST(Predicate2dTest, KeepsBoundaryAndConstexprContracts)
{
    static_assert(Geometry::IsZero(0.0, 0.0));
    static_assert(!Geometry::IsZero(0.25, 0.0));
    static_assert(Geometry::IsZero(-0.25, 0.25));
    static_assert(Geometry::IsEqual(0.5, 0.75, 0.25));
    static_assert(!Geometry::IsEqual(0.5, 0.75, 0.125));
    // 整数入口仍精确比较，忽略不参与其语义的浮点容差。
    static_assert(Geometry::IsZero(0, -1.0));
    static_assert(Geometry::IsEqual(2, 2, -1.0));
    EXPECT_TRUE(Geometry::IsEqual(Geometry::SCPoint2d{0.5, 0.75}, Geometry::SCPoint2d{0.75, 0.5}, 0.25));
    EXPECT_FALSE(Geometry::IsEqual(Geometry::SCPoint2d{0.5, 0.75}, Geometry::SCPoint2d{0.75, 0.5}, 0.125));
    EXPECT_TRUE(Geometry::IsZero(Geometry::SCVector2i{}, -1.0));
}

TEST(Predicate2dTest, FloatPredicatesPreserveDoubleToleranceBoundary)
{
    const double belowOne = std::nextafter(1.0, 0.0);
    EXPECT_FALSE(Geometry::IsZero(1.0f, belowOne));
    EXPECT_FALSE(Geometry::IsEqual(0.0f, 1.0f, belowOne));
    EXPECT_TRUE(Geometry::IsZero(1.0f, 1.0));
    EXPECT_TRUE(Geometry::IsEqual(0.0f, 1.0f, 1.0));
    const float maximum = (std::numeric_limits<float>::max)();
    const double separation = 2.0 * static_cast<double>(maximum);
    EXPECT_TRUE(Geometry::IsEqual(-maximum, maximum, separation));
    EXPECT_FALSE(Geometry::IsEqual(-maximum, maximum, std::nextafter(separation, 0.0)));
}
