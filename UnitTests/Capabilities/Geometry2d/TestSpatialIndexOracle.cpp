#include <algorithm>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "Geometry2d/SCBoxTree2d.h"
#include "Geometry2d/SCKDTree2d.h"
#include "Types/Geometry2d/SCBox2.h"

namespace
{
    std::vector<std::size_t> Sorted(std::vector<std::size_t> values)
    {
        std::sort(values.begin(), values.end());
        return values;
    }
}

TEST(SpatialIndexOracleTest, KdTreeQueriesMatchBruteForceForDeterministicRandomData)
{
    std::mt19937 generator(20260901U);
    std::uniform_real_distribution<double> coordinate(-10.0, 10.0);
    Geometry::SCKDTree2d tree;
    std::vector<Geometry::SCKDTreeEntry2d> entries;
    for (std::size_t id = 0; id < 64; ++id)
    {
        const Geometry::SCPoint2d point{coordinate(generator), coordinate(generator)};
        tree.Add(id, point);
        entries.push_back({id, point});
    }

    for (int sample = 0; sample < 16; ++sample)
    {
        const Geometry::SCPoint2d query{coordinate(generator), coordinate(generator)};
        const double radius = 3.0;
        std::vector<std::size_t> expected;
        for (const auto& entry : entries)
        {
            const double dx = entry.point.x - query.x;
            const double dy = entry.point.y - query.y;
            if (dx * dx + dy * dy <= radius * radius)
            {
                expected.push_back(entry.id);
            }
        }
        EXPECT_EQ(Sorted(tree.QueryWithinDistance(query, radius)), Sorted(expected));
    }
}

TEST(SpatialIndexOracleTest, BoxTreeKNearestMatchesBruteForceAndMutationsAreImmediatelyVisible)
{
    Geometry::SCBoxTree2d tree;
    tree.Add(9, Geometry::SCBox2d::FromMinMax({4.0, 0.0}, {5.0, 1.0}));
    tree.Add(3, Geometry::SCBox2d::FromMinMax({1.0, 0.0}, {2.0, 1.0}));
    tree.Add(7, Geometry::SCBox2d::FromMinMax({2.0, 0.0}, {3.0, 1.0}));

    const auto initialHits = tree.QueryKNearest({0.0, 0.0}, 3);
    ASSERT_EQ(initialHits.size(), 3U);
    EXPECT_EQ(initialHits[0].id, 3U);
    EXPECT_EQ(initialHits[1].id, 7U);
    EXPECT_EQ(initialHits[2].id, 9U);

    tree.Update(9, Geometry::SCBox2d::FromMinMax({-1.0, 0.0}, {0.0, 1.0}));
    EXPECT_EQ(tree.QueryKNearest({0.0, 0.0}, 1)[0].id, 9U);
    ASSERT_TRUE(tree.Remove(9));
    EXPECT_EQ(tree.QueryKNearest({0.0, 0.0}, 1)[0].id, 3U);
    tree.Add(1, Geometry::SCBox2d::FromMinMax({-0.5, 0.0}, {0.0, 1.0}));
    EXPECT_EQ(tree.QueryKNearest({0.0, 0.0}, 1)[0].id, 1U);
}
