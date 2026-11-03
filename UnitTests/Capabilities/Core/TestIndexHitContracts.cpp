#include <gtest/gtest.h>

#include <limits>

#include "Geometry2d/SCSegmentSearch2d.h"
#include "Geometry3d/SCSegmentSearch3d.h"

namespace
{
    Geometry::SCSegmentSearch2d UnderflowSegmentSearch()
    {
        Geometry::SCSegmentSearch2d search;
        search.Add(Geometry::SCLineSegment2d({2e-200, -1}, {2e-200, 1}));
        search.Add(Geometry::SCLineSegment2d({1e-200, -1}, {1e-200, 1}));
        return search;
    }

    template <typename Hit>
    void CheckSegmentSearchHitPayload()
    {
        Hit hit{};
        hit.distanceSquared = 4.0;
        hit.parameter = 0.5;
        ASSERT_TRUE(hit.IsValid());
        const double infinity = std::numeric_limits<double>::infinity();
        const double nan = std::numeric_limits<double>::quiet_NaN();
        for (const double invalid : {-1.0, infinity, -infinity, nan})
        {
            hit.distanceSquared = invalid;
            EXPECT_FALSE(hit.IsValid());
        }
        hit.distanceSquared = 0.0;
        for (const double invalid : {infinity, -infinity, nan})
        {
            hit.parameter = invalid;
            EXPECT_FALSE(hit.IsValid());
        }
        // 参数域由具体曲线决定，有效性不强制限制到 [0, 1]。
        hit.parameter = -2.0;
        EXPECT_TRUE(hit.IsValid());
        hit.parameter = 5.0;
        EXPECT_TRUE(hit.IsValid());
        hit.point.x = nan;
        EXPECT_FALSE(hit.IsValid());
    }
}

TEST(IndexHitContractTest, SegmentSearch2dRejectsNonFinitePayloads)
{
    CheckSegmentSearchHitPayload<Geometry::SCSegmentSearchHit2d>();
}

TEST(IndexHitContractTest, SegmentSearch3dRejectsNonFinitePayloads)
{
    CheckSegmentSearchHitPayload<Geometry::SCSegmentSearchHit3d>();
}

TEST(IndexHitContractTest, SegmentSearchRadiusUsesDistanceWhenSquaresUnderflow)
{
    const auto search = UnderflowSegmentSearch();
    EXPECT_TRUE(search.QueryWithinDistance({0, 0}, 0.0).empty());
    const auto hits = search.QueryWithinDistance({0, 0}, 1.5e-200);
    ASSERT_EQ(hits.size(), 1U);
    EXPECT_EQ(hits[0].id, 1U);
    EXPECT_TRUE(hits[0].IsValid());
}

TEST(IndexHitContractTest, SegmentSearchNearestUsesDistanceWhenSquaresUnderflow)
{
    const auto search = UnderflowSegmentSearch();
    const auto hit = search.Nearest({0, 0});
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->id, 1U);
    EXPECT_TRUE(hit->IsValid());
}

TEST(IndexHitContractTest, SegmentSearchKnnUsesDistanceWhenSquaresUnderflow)
{
    const auto search = UnderflowSegmentSearch();
    const auto hits = search.QueryKNearest({0, 0}, 2);
    ASSERT_EQ(hits.size(), 2U);
    EXPECT_EQ(hits[0].id, 1U);
    EXPECT_EQ(hits[1].id, 0U);
    const auto nearest = search.QueryKNearest({0, 0}, 1);
    ASSERT_EQ(nearest.size(), 1U);
    EXPECT_EQ(nearest[0].id, 1U);
    const auto bounded = search.QueryKNearest({0, 0}, 2, 1.5e-200);
    ASSERT_EQ(bounded.size(), 1U);
    EXPECT_EQ(bounded[0].id, 1U);
    EXPECT_TRUE(search.QueryKNearest({0, 0}, 0).empty());
}
