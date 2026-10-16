#include <gtest/gtest.h>

#include "Geometry.h"

TEST(DISABLED_MissingAlgorithms3dGapTest, ProjectPointToLineSegmentToleranceBoundaryRemainsOpen)
{
    GTEST_SKIP() << "Known gap: zero-length segment projection tolerance boundary.";
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
