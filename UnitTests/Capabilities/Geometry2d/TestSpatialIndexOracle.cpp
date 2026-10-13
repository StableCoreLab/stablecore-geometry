#include <algorithm>
#include <limits>
#include <random>
#include <type_traits>
#include <utility>
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

TEST(SpatialIndexOracleTest, IncrementalBoxTreesKeepQueriesConsistentAfterRemovalAndMove)
{
    static_assert(std::is_const_v<std::remove_reference_t<decltype(std::declval<Geometry::SCBoxTree2d&>().Entries())>>);
    static_assert(std::is_const_v<std::remove_reference_t<decltype(std::declval<Geometry::SCBoxTree2d&>().Entries())>>);

    Geometry::SCBoxTree2d tree;
    for (std::size_t id = 0; id < 12; ++id)
    {
        tree.Add(id, Geometry::SCBox2d::FromMinMax({static_cast<double>(id), 0.0},
                                                   {static_cast<double>(id) + 0.5, 0.5}));
    }
    tree.Add(3, Geometry::SCBox2d::FromMinMax({30.0, 0.0}, {30.5, 0.5}));
    EXPECT_TRUE(tree.Remove(3));
    EXPECT_EQ(tree.Size(), 11U);
    tree.Update(10, Geometry::SCBox2d::FromMinMax({-2.0, 0.0}, {-1.5, 0.5}));
    EXPECT_EQ(tree.QueryContaining({-1.75, 0.25}), std::vector<std::size_t>{10});

    Geometry::SCBoxTree2d copied = tree;
    Geometry::SCBoxTree2d moved = std::move(copied);
    Geometry::SCBoxTree2d assigned;
    assigned = std::move(moved);
    const auto hits = assigned.QueryKNearest({-2.0, 0.0}, 1);
    ASSERT_EQ(hits.size(), 1U);
    EXPECT_EQ(hits.front().id, 10U);
    EXPECT_EQ(assigned.QueryKNearest({-2.0, 0.0}, 0).size(), 0U);
    for (std::size_t id = 0; id < 12; ++id)
    {
        (void)assigned.Remove(id);
    }
    EXPECT_TRUE(assigned.QueryKNearest({0.0, 0.0}, 1).empty());
    assigned.Add(42, Geometry::SCBox2d::FromMinMax({0.0, 0.0}, {1.0, 1.0}));
    EXPECT_EQ(assigned.QueryContaining({0.5, 0.5}), std::vector<std::size_t>{42});
}

TEST(SpatialIndexOracleTest, IncrementalKdTreeTracksUpdatesAndDuplicateIds)
{
    static_assert(std::is_const_v<std::remove_reference_t<decltype(std::declval<Geometry::SCKDTree2d&>().Entries())>>);
    Geometry::SCKDTree2d tree;
    for (std::size_t id = 0; id < 12; ++id)
    {
        tree.Add(id, {static_cast<double>(id), 0.0});
    }
    tree.Add(3, {30.0, 0.0});
    EXPECT_TRUE(tree.Remove(3));
    tree.Update(10, {-2.0, 0.0});
    EXPECT_EQ(tree.Query({-2.0, 0.0}, 0.0), std::vector<std::size_t>{10});
    ASSERT_TRUE(tree.Nearest({-2.0, 0.0}).has_value());
    EXPECT_EQ(tree.Nearest({-2.0, 0.0})->id, 10U);
    Geometry::SCKDTree2d copied = tree;
    Geometry::SCKDTree2d moved = std::move(copied);
    Geometry::SCKDTree2d assigned;
    assigned = std::move(moved);
    EXPECT_EQ(assigned.QueryWithinDistance({-2.0, 0.0}, 0.5), std::vector<std::size_t>{10});
}

TEST(SpatialIndexOracleTest, BoxTreeApproximateKnnRespectsRelativeDistanceBound)
{
    Geometry::SCBoxTree2d tree;
    std::vector<std::pair<double, std::size_t>> expected;
    for (std::size_t id = 0; id < 24; ++id)
    {
        const double x = static_cast<double>(id % 6) - 3.0;
        const double y = static_cast<double>(id / 6) - 2.0;
        tree.Add(id, Geometry::SCBox2d::FromMinMax({x, y}, {x + 0.25, y + 0.25}));
        const double dx = std::max({x - 0.1, 0.0, 0.1 - (x + 0.25)});
        const double dy = std::max({y - 0.2, 0.0, 0.2 - (y + 0.25)});
        expected.emplace_back(dx * dx + dy * dy, id);
    }
    std::sort(expected.begin(), expected.end());
    for (std::size_t k : {1U, 3U, 8U})
    {
        const auto exact = tree.QueryKNearest({0.1, 0.2}, k, 0.0);
        const auto approximate = tree.QueryKNearest({0.1, 0.2}, k, 0.25);
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

TEST(SpatialIndexOracleTest, InvalidEntriesRemainInspectableWithoutEnteringTheTrees)
{
    Geometry::SCBoxTree2d boxes;
    for (std::size_t id = 0; id < 6; ++id)
    {
        boxes.Add(id, Geometry::SCBox2d{});
    }
    EXPECT_EQ(boxes.Size(), 6U);
    EXPECT_FALSE(boxes.IsValid());
    EXPECT_TRUE(boxes.QueryKNearest({0.0, 0.0}, 1).empty());
    boxes.Update(4, Geometry::SCBox2d::FromMinMax({1.0, 1.0}, {2.0, 2.0}));
    EXPECT_EQ(boxes.QueryContaining({1.5, 1.5}), std::vector<std::size_t>{4});
    boxes.Update(4, Geometry::SCBox2d{});
    EXPECT_TRUE(boxes.QueryContaining({1.5, 1.5}).empty());

    Geometry::SCKDTree2d points;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    points.Add(1, {nan, 0.0});
    points.Add(2, {1.0, 1.0});
    EXPECT_FALSE(points.IsValid());
    EXPECT_EQ(points.Query({1.0, 1.0}, 0.0), std::vector<std::size_t>{2});
    points.Update(1, {1.0, 1.0});
    EXPECT_TRUE(points.IsValid());
    EXPECT_EQ(Sorted(points.Query({1.0, 1.0}, 0.0)), (std::vector<std::size_t>{1, 2}));
}

TEST(SpatialIndexOracleTest, AddAcceptsGeometryReferencedFromExistingEntry)
{
    Geometry::SCBoxTree2d boxes;
    const auto box = Geometry::SCBox2d::FromMinMax({1.0, 1.0}, {2.0, 2.0});
    boxes.Add(0, box);
    while (boxes.Size() < boxes.Entries().capacity())
    {
        boxes.Add(boxes.Size(), box);
    }
    boxes.Add(99, boxes.Entries().front().box);
    EXPECT_EQ(boxes.QueryContaining({1.5, 1.5}).size(), boxes.Size());
    ASSERT_NE(boxes.Find(99), nullptr);
    EXPECT_TRUE(boxes.Find(99)->box.IsValid());

    Geometry::SCKDTree2d points;
    points.Add(0, {1.0, 1.0});
    while (points.Size() < points.Entries().capacity())
    {
        points.Add(points.Size(), {1.0, 1.0});
    }
    points.Add(99, points.Entries().front().point);
    EXPECT_EQ(points.Query({1.0, 1.0}, 0.0).size(), points.Size());
    ASSERT_NE(points.Find(99), nullptr);
    EXPECT_TRUE(points.Find(99)->point.IsValid());
}

TEST(SpatialIndexOracleTest, RemoveManyDuplicateIdsKeepsRemainingIndicesQueryable)
{
    Geometry::SCBoxTree2d boxes;
    boxes.Add(1, Geometry::SCBox2d::FromMinMax({-2.0, 0.0}, {-1.0, 1.0}));
    for (std::size_t index = 0; index < 32; ++index)
    {
        const double x = static_cast<double>(index);
        boxes.Add(7, Geometry::SCBox2d::FromMinMax({x, 0.0}, {x + 0.5, 1.0}));
    }
    boxes.Add(2, Geometry::SCBox2d::FromMinMax({40.0, 0.0}, {41.0, 1.0}));
    EXPECT_TRUE(boxes.Remove(7));
    EXPECT_EQ(boxes.Size(), 2U);
    EXPECT_EQ(boxes.QueryContaining({-1.5, 0.5}), std::vector<std::size_t>{1});
    EXPECT_EQ(boxes.QueryContaining({40.5, 0.5}), std::vector<std::size_t>{2});

    Geometry::SCKDTree2d points;
    points.Add(1, {-2.0, 0.0});
    for (std::size_t index = 0; index < 32; ++index)
    {
        points.Add(7, {static_cast<double>(index), 0.0});
    }
    points.Add(2, {40.0, 0.0});
    EXPECT_TRUE(points.Remove(7));
    EXPECT_EQ(points.Size(), 2U);
    EXPECT_EQ(points.Query({-2.0, 0.0}, 0.0), std::vector<std::size_t>{1});
    EXPECT_EQ(points.Query({40.0, 0.0}, 0.0), std::vector<std::size_t>{2});
}
