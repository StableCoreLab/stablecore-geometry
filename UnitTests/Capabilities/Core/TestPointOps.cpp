#include <gtest/gtest.h>

#include <type_traits>

#include "Core/PointOps.h"

using Geometry::Midpoint;
using Geometry::SCPoint2d;
using Geometry::SCPoint3d;

TEST(PointOpsTest, ComputesTwoDimensionalMidpoint)
{
    const SCPoint2d result = Midpoint(SCPoint2d{0.0, 0.0}, SCPoint2d{10.0, 20.0});
    EXPECT_DOUBLE_EQ(result.x, 5.0);
    EXPECT_DOUBLE_EQ(result.y, 10.0);
}

TEST(PointOpsTest, ComputesAllThreeDimensions)
{
    const SCPoint3d result = Midpoint(SCPoint3d{1.0, 2.0, 3.0}, SCPoint3d{5.0, 8.0, 13.0});
    EXPECT_DOUBLE_EQ(result.x, 3.0);
    EXPECT_DOUBLE_EQ(result.y, 5.0);
    EXPECT_DOUBLE_EQ(result.z, 8.0);
}

TEST(PointOpsTest, PreservesNegativeAndFractionalCoordinates)
{
    const SCPoint2d result = Midpoint(SCPoint2d{-3.5, 4.25}, SCPoint2d{8.5, -1.75});
    EXPECT_DOUBLE_EQ(result.x, 2.5);
    EXPECT_DOUBLE_EQ(result.y, 1.25);
}

TEST(PointOpsTest, HandlesCoincidentPoints)
{
    const SCPoint3d point{1.25, -2.5, 1.0e300};
    const SCPoint3d result = Midpoint(point, point);
    EXPECT_DOUBLE_EQ(result.x, point.x);
    EXPECT_DOUBLE_EQ(result.y, point.y);
    EXPECT_DOUBLE_EQ(result.z, point.z);
}

TEST(PointOpsTest, IsSymmetricInItsInputs)
{
    const SCPoint3d first{-10.0, 2.5, 100.0};
    const SCPoint3d second{6.0, -4.5, 20.0};
    const SCPoint3d forward = Midpoint(first, second);
    const SCPoint3d reverse = Midpoint(second, first);
    EXPECT_DOUBLE_EQ(forward.x, reverse.x);
    EXPECT_DOUBLE_EQ(forward.y, reverse.y);
    EXPECT_DOUBLE_EQ(forward.z, reverse.z);
}

TEST(PointOpsTest, RetainsExpectedFloatingPointPrecisionForLargeCoordinates)
{
    const SCPoint2d result = Midpoint(SCPoint2d{1.0e12, -1.0e12}, SCPoint2d{1.0e12 + 2.0, -1.0e12 + 2.0});
    EXPECT_DOUBLE_EQ(result.x, 1.0e12 + 1.0);
    EXPECT_DOUBLE_EQ(result.y, -1.0e12 + 1.0);
}

TEST(PointOpsTest, IsNoexceptAndUsesOnlyPublicGeometryTypes)
{
    static_assert(noexcept(Midpoint(SCPoint2d{}, SCPoint2d{})));
    static_assert(noexcept(Midpoint(SCPoint3d{}, SCPoint3d{})));
    static_assert(std::is_same_v<decltype(Midpoint(SCPoint2d{}, SCPoint2d{})), SCPoint2d>);
    static_assert(std::is_same_v<decltype(Midpoint(SCPoint3d{}, SCPoint3d{})), SCPoint3d>);
}
