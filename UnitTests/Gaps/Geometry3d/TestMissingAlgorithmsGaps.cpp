#include <gtest/gtest.h>

#include <limits>

#include "Geometry.h"

TEST(MissingAlgorithms3dGapTest, ProjectPointToLineSegmentRespectsTinyToleranceBoundary)
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

TEST(DISABLED_MissingAlgorithms3dGapTest, SnapPointToSegments3dEqualDistanceTieBreakingRemainsOpen)
{
    GTEST_SKIP() << "Known gap: equal-distance 3D snap tie-breaking contract.";
}

TEST(MissingAlgorithms3dGapTest, SegmentSearch3dKNearestBeyondRepresentativeCases)
{
    using namespace Geometry;
    SCSegmentSearch3d search;
    const SCLineSegment3d first{{-1.0, 1.0, 0.0}, {1.0, 1.0, 0.0}};
    const SCLineSegment3d second{{-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}};
    const SCLineSegment3d outside{{-1.0, 3.0, 0.0}, {1.0, 3.0, 0.0}};
    search.Add(first);
    search.Add(second);
    search.Add(outside);

    const auto hits = search.QueryKNearest(SCPoint3d{0.0, 0.0, 0.0}, 3, 1.5);
    ASSERT_EQ(hits.size(), 2U);
    EXPECT_EQ(hits[0].id, 0U);
    EXPECT_EQ(hits[1].id, 1U);
    EXPECT_TRUE(search.QueryKNearest(SCPoint3d{0.0, 0.0, 0.0}, 2, 0.5).empty());
    EXPECT_EQ(search.QueryWithinDistance(SCPoint3d{0.0, 0.0, 0.0}, 1.5).size(), 2U);
}
