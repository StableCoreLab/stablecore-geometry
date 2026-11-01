#include <gtest/gtest.h>

#include <algorithm>
#include <utility>
#include <vector>

#include "Geometry.h"

using Geometry::SCBox2d;
using Geometry::SCBox3d;
using Geometry::SCBoxTree2d;
using Geometry::SCBoxTree3d;
using Geometry::SCLineSegment2d;
using Geometry::SCLineSegment3d;
using Geometry::SCPoint2d;
using Geometry::SCPoint3d;
using Geometry::SCSegmentSearch2d;
using Geometry::SCSegmentSearch3d;

TEST(AlgorithmCapabilitiesTest, QueryKNearestReturnsOrderedHits)
{
    SCBoxTree2d tree2d;
    tree2d.Add(1, SCBox2d::FromMinMax(SCPoint2d{2.0, 2.0}, SCPoint2d{3.0, 3.0}));
    tree2d.Add(2, SCBox2d::FromMinMax(SCPoint2d{0.5, 0.5}, SCPoint2d{1.0, 1.0}));
    tree2d.Add(3, SCBox2d::FromMinMax(SCPoint2d{5.0, 5.0}, SCPoint2d{6.0, 6.0}));

    const auto hits2d = tree2d.QueryKNearest(SCPoint2d{0.0, 0.0}, 2);
    ASSERT_EQ(hits2d.size(), 2U);
    EXPECT_EQ(hits2d[0].id, 2U);
    EXPECT_EQ(hits2d[1].id, 1U);

    SCSegmentSearch2d search2d;
    search2d.Add(SCLineSegment2d{SCPoint2d{0.0, 0.0}, SCPoint2d{1.0, 0.0}});
    search2d.Add(SCLineSegment2d{SCPoint2d{3.0, 0.0}, SCPoint2d{4.0, 0.0}});
    const auto hitsSearch2d = search2d.QueryKNearest(SCPoint2d{0.5, 0.5}, 1);
    ASSERT_EQ(hitsSearch2d.size(), 1U);
    EXPECT_EQ(hitsSearch2d[0].id, 0U);

    SCBoxTree3d tree3d;
    tree3d.Add(7, SCBox3d::FromMinMax(SCPoint3d{2.0, 2.0, 2.0}, SCPoint3d{3.0, 3.0, 3.0}));
    tree3d.Add(8, SCBox3d::FromMinMax(SCPoint3d{0.5, 0.5, 0.5}, SCPoint3d{1.0, 1.0, 1.0}));
    const auto hits3d = tree3d.QueryKNearest(SCPoint3d{0.0, 0.0, 0.0}, 1);
    ASSERT_EQ(hits3d.size(), 1U);
    EXPECT_EQ(hits3d[0].id, 8U);

    SCSegmentSearch3d search3d;
    search3d.Add(SCLineSegment3d{SCPoint3d{0.0, 0.0, 0.0}, SCPoint3d{1.0, 0.0, 0.0}});
    search3d.Add(SCLineSegment3d{SCPoint3d{3.0, 0.0, 0.0}, SCPoint3d{4.0, 0.0, 0.0}});
    const auto hitsSearch3d = search3d.QueryKNearest(SCPoint3d{0.25, 0.0, 0.0}, 1);
    ASSERT_EQ(hitsSearch3d.size(), 1U);
    EXPECT_EQ(hitsSearch3d[0].id, 0U);
}

TEST(AlgorithmCapabilitiesTest, QueryKNearestBreaksEqualDistanceTiesById)
{
    SCBoxTree2d tree2d;
    tree2d.Add(30, SCBox2d::FromMinMax(SCPoint2d{2.0, 0.0}, SCPoint2d{3.0, 1.0}));
    tree2d.Add(10, SCBox2d::FromMinMax(SCPoint2d{-3.0, 0.0}, SCPoint2d{-2.0, 1.0}));
    tree2d.Add(20, SCBox2d::FromMinMax(SCPoint2d{-0.5, 2.0}, SCPoint2d{0.5, 3.0}));

    const auto hits2d = tree2d.QueryKNearest(SCPoint2d{0.0, 0.0}, 2);
    ASSERT_EQ(hits2d.size(), 2U);
    EXPECT_EQ(hits2d[0].id, 10U);
    EXPECT_EQ(hits2d[1].id, 20U);
    EXPECT_DOUBLE_EQ(hits2d[0].distanceSquared, hits2d[1].distanceSquared);

    SCBoxTree3d tree3d;
    tree3d.Add(30, SCBox3d::FromMinMax(SCPoint3d{2.0, 0.0, 0.0}, SCPoint3d{3.0, 1.0, 1.0}));
    tree3d.Add(10, SCBox3d::FromMinMax(SCPoint3d{-3.0, 0.0, 0.0}, SCPoint3d{-2.0, 1.0, 1.0}));
    tree3d.Add(20, SCBox3d::FromMinMax(SCPoint3d{-0.5, 2.0, 0.0}, SCPoint3d{0.5, 3.0, 1.0}));

    const auto hits3d = tree3d.QueryKNearest(SCPoint3d{0.0, 0.0, 0.0}, 2);
    ASSERT_EQ(hits3d.size(), 2U);
    EXPECT_EQ(hits3d[0].id, 10U);
    EXPECT_EQ(hits3d[1].id, 20U);
    EXPECT_DOUBLE_EQ(hits3d[0].distanceSquared, hits3d[1].distanceSquared);
}

TEST(AlgorithmCapabilitiesTest, BoxTree3dKeepsIncrementalQueriesConsistent)
{
    SCBoxTree3d tree;
    tree.Add(99, SCBox3d{});
    EXPECT_FALSE(tree.IsValid());
    for (std::size_t id = 0; id < 12; ++id)
    {
        const double x = static_cast<double>(id);
        tree.Add(id, SCBox3d::FromMinMax({x, 0.0, 0.0}, {x + 0.5, 0.5, 0.5}));
    }
    tree.Add(3, SCBox3d::FromMinMax({30.0, 0.0, 0.0}, {30.5, 0.5, 0.5}));
    EXPECT_TRUE(tree.Remove(3));
    tree.Update(10, SCBox3d::FromMinMax({-2.0, 0.0, 0.0}, {-1.5, 0.5, 0.5}));
    tree.Update(99, SCBox3d::FromMinMax({-4.0, 0.0, 0.0}, {-3.5, 0.5, 0.5}));
    EXPECT_TRUE(tree.IsValid());
    EXPECT_EQ(tree.QueryContaining({-1.75, 0.25, 0.25}), std::vector<std::size_t>{10});
    SCBoxTree3d copied = tree;
    SCBoxTree3d moved = std::move(copied);
    SCBoxTree3d assigned;
    assigned = std::move(moved);
    const auto hits = assigned.QueryKNearest({-2.0, 0.0, 0.0}, 1);
    ASSERT_EQ(hits.size(), 1U);
    EXPECT_EQ(hits.front().id, 10U);
}

TEST(AlgorithmCapabilitiesTest, BoxTree3dKnnMatchesIndependentDistanceOracle)
{
    SCBoxTree3d tree;
    std::vector<std::pair<double, std::size_t>> expected;
    for (std::size_t id = 0; id < 18; ++id)
    {
        const double x = static_cast<double>(id % 3) - 1.0;
        const double y = static_cast<double>((id / 3) % 3) - 1.0;
        const double z = static_cast<double>(id / 9) - 0.5;
        tree.Add(id, SCBox3d::FromMinMax({x, y, z}, {x + 0.25, y + 0.25, z + 0.25}));
        const double dx = std::max({x - 0.1, 0.0, 0.1 - (x + 0.25)});
        const double dy = std::max({y - 0.2, 0.0, 0.2 - (y + 0.25)});
        const double dz = std::max({z - 0.3, 0.0, 0.3 - (z + 0.25)});
        expected.emplace_back(dx * dx + dy * dy + dz * dz, id);
    }
    std::sort(expected.begin(), expected.end());
    for (std::size_t k : {1U, 4U, 9U})
    {
        const auto exact = tree.QueryKNearest({0.1, 0.2, 0.3}, k, 0.0);
        const auto approximate = tree.QueryKNearest({0.1, 0.2, 0.3}, k, 0.25);
        ASSERT_EQ(exact.size(), k);
        ASSERT_EQ(approximate.size(), k);
        for (std::size_t index = 0; index < k; ++index)
        {
            EXPECT_EQ(exact[index].id, expected[index].second);
            EXPECT_DOUBLE_EQ(exact[index].distanceSquared, expected[index].first);
        }
        EXPECT_LE(approximate.back().distanceSquared, expected[k - 1].first * 1.25 * 1.25);
    }
}

TEST(AlgorithmCapabilitiesTest, BoxTree3dAddAcceptsGeometryReferencedFromExistingEntry)
{
    SCBoxTree3d tree;
    const auto box = SCBox3d::FromMinMax({1.0, 1.0, 1.0}, {2.0, 2.0, 2.0});
    tree.Add(0, box);
    while (tree.Size() < tree.Entries().capacity())
    {
        tree.Add(tree.Size(), box);
    }
    tree.Add(99, tree.Entries().front().box);
    EXPECT_EQ(tree.QueryContaining({1.5, 1.5, 1.5}).size(), tree.Size());
    ASSERT_NE(tree.Find(99), nullptr);
    EXPECT_TRUE(tree.Find(99)->box.IsValid());
}

TEST(AlgorithmCapabilitiesTest, BoxTree3dRemovesManyDuplicateIdsInOnePass)
{
    SCBoxTree3d tree;
    tree.Add(1, SCBox3d::FromMinMax({-2.0, 0.0, 0.0}, {-1.0, 1.0, 1.0}));
    for (std::size_t index = 0; index < 32; ++index)
    {
        const double x = static_cast<double>(index);
        tree.Add(7, SCBox3d::FromMinMax({x, 0.0, 0.0}, {x + 0.5, 1.0, 1.0}));
    }
    tree.Add(2, SCBox3d::FromMinMax({40.0, 0.0, 0.0}, {41.0, 1.0, 1.0}));
    EXPECT_TRUE(tree.Remove(7));
    EXPECT_EQ(tree.Size(), 2U);
    EXPECT_EQ(tree.QueryContaining({-1.5, 0.5, 0.5}), std::vector<std::size_t>{1});
    EXPECT_EQ(tree.QueryContaining({40.5, 0.5, 0.5}), std::vector<std::size_t>{2});
}
