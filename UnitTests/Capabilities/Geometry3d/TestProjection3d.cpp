#include <gtest/gtest.h>

#include <limits>

#include "Core/Projection.h"
#include "Types/Geometry3d/SCGeometryTypes3d.h"
#include "Types/Geometry3d/SCLineSegment3d.h"

TEST(Projection3dTest, ProjectPointToLineSegmentRespectsTinyToleranceBoundary)
{
    using namespace Geometry;
    const SCLineSegment3d tiny{{0.0, 0.0, 0.0}, {1e-200, 0.0, 0.0}};
    SCGeometryTolerance3d tolerance;
    tolerance.distanceEpsilon = 1e-210;
    EXPECT_TRUE(tiny.IsValid(tolerance.distanceEpsilon));
    EXPECT_TRUE(tiny.Direction(tolerance.distanceEpsilon).IsValid());
    EXPECT_EQ(tiny.PointAtLength(5e-201), tiny.startPoint);
    EXPECT_EQ(tiny.PointAtLength(5e-201, false, tolerance.distanceEpsilon),
              (SCPoint3d{5e-201, 0.0, 0.0}));
    EXPECT_EQ(tiny.PointAtLength(2e-200, true, tolerance.distanceEpsilon), tiny.endPoint);
    const auto projection = ProjectPointToLineSegment({5e-201, 1.0, 0.0}, tiny, true, tolerance);
    ASSERT_TRUE(projection.IsValid());
    EXPECT_DOUBLE_EQ(projection.parameter, 0.5);
    EXPECT_EQ(projection.point, (SCPoint3d{5e-201, 0.0, 0.0}));
    EXPECT_TRUE(projection.isOnSegment);

    tolerance.distanceEpsilon = 1e-200;
    const auto boundary = ProjectPointToLineSegment({5e-201, 1.0, 0.0}, tiny, true, tolerance);
    ASSERT_TRUE(boundary.IsValid());
    EXPECT_EQ(boundary.point, tiny.startPoint);
    EXPECT_DOUBLE_EQ(boundary.parameter, 0.0);
    EXPECT_FALSE(tiny.IsValid(tolerance.distanceEpsilon));

    const SCLineSegment3d zero{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    EXPECT_EQ(ProjectPointToLineSegment({1.0, 0.0, 0.0}, zero, true, tolerance).point, zero.startPoint);
    tolerance.distanceEpsilon = -1.0;
    EXPECT_FALSE(ProjectPointToLineSegment({0.0, 0.0, 0.0}, zero, true, tolerance).IsValid());
    EXPECT_EQ(tiny.PointAtLength(5e-201, false, tolerance.distanceEpsilon), tiny.startPoint);

    const double largest = std::numeric_limits<double>::max();
    const SCLineSegment3d unrepresentable{{-largest, 0.0, 0.0}, {largest, 0.0, 0.0}};
    EXPECT_FALSE(unrepresentable.IsValid());
    EXPECT_FALSE(ProjectPointToLineSegment({0.0, 0.0, 0.0}, unrepresentable, true).IsValid());
}
