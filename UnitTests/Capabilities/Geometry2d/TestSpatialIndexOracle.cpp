#include <algorithm>
#include <atomic>
#include <limits>
#include <random>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "Core/Metrics.h"
#include "Geometry2d/SCBoxTree2d.h"
#include "Geometry2d/SCKDTree2d.h"
#include "Geometry3d/SCBoxTree3d.h"
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

TEST(SpatialIndexOracleTest, KdTreeDistanceQueriesPreserveOrderOutsideSquaredRange)
{
    Geometry::SCKDTree2d tree;
    tree.Add(2, {2e200, 0.0});
    tree.Add(1, {1e200, 0.0});
    ASSERT_TRUE(tree.Nearest({0.0, 0.0}).has_value());
    EXPECT_EQ(tree.Nearest({0.0, 0.0})->id, 1U);
    EXPECT_EQ(Sorted(tree.QueryWithinDistance({0.0, 0.0}, 1.5e200)),
              std::vector<std::size_t>{1});

    tree.Clear();
    tree.Add(2, {2e-200, 0.0});
    tree.Add(1, {1e-200, 0.0});
    ASSERT_TRUE(tree.Nearest({0.0, 0.0}).has_value());
    EXPECT_EQ(tree.Nearest({0.0, 0.0})->id, 1U);
    EXPECT_EQ(Sorted(tree.QueryWithinDistance({0.0, 0.0}, 1.5e-200)),
              std::vector<std::size_t>{1});
    EXPECT_TRUE(tree.QueryWithinDistance({0.0, 0.0}, -1.5e-200).empty());
}

TEST(SpatialIndexOracleTest, KdTreeNearestOrdersOverflowingCoordinateDifferences)
{
    Geometry::SCKDTree2d tree;
    tree.Add(2, {-1e308, 0.0});
    tree.Add(1, {-8e307, 0.0});
    const auto nearest = tree.Nearest({1e308, 0.0});
    ASSERT_TRUE(nearest.has_value());
    EXPECT_EQ(nearest->id, 1U);

    tree.Clear();
    tree.Add(2, {-9e307, 1e308});
    tree.Add(3, {-8e307, 1e308});
    tree.Add(1, {-1e308, 0.0});
    const auto acrossSplit = tree.Nearest({1e308, 0.0});
    ASSERT_TRUE(acrossSplit.has_value());
    EXPECT_EQ(acrossSplit->id, 1U);
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

TEST(SpatialIndexOracleTest, BoxTreeKNearestOrdersDistancesOutsideSquaredRange)
{
    Geometry::SCBoxTree2d boxes2d;
    boxes2d.Add(1, Geometry::SCBox2d::FromMinMax({2e200, 0.0}, {2e200, 1.0}));
    boxes2d.Add(9, Geometry::SCBox2d::FromMinMax({1e200, 0.0}, {1e200, 1.0}));
    ASSERT_EQ(boxes2d.QueryKNearest({0.0, 0.0}, 2).size(), 2U);
    EXPECT_EQ(boxes2d.QueryKNearest({0.0, 0.0}, 2)[0].id, 9U);
    EXPECT_EQ(boxes2d.QueryKNearest({0.0, 0.0}, 1)[0].id, 9U);

    Geometry::SCBoxTree3d boxes3d;
    boxes3d.Add(1, Geometry::SCBox3d::FromMinMax({2e-200, 0.0, 0.0}, {2e-200, 1.0, 1.0}));
    boxes3d.Add(9, Geometry::SCBox3d::FromMinMax({1e-200, 0.0, 0.0}, {1e-200, 1.0, 1.0}));
    ASSERT_EQ(boxes3d.QueryKNearest({0.0, 0.0, 0.0}, 2).size(), 2U);
    EXPECT_EQ(boxes3d.QueryKNearest({0.0, 0.0, 0.0}, 2)[0].id, 9U);
    EXPECT_EQ(boxes3d.QueryKNearest({0.0, 0.0, 0.0}, 1)[0].id, 9U);

    Geometry::SCBoxTree2d branched2d;
    Geometry::SCBoxTree3d branched3d;
    for (std::size_t id = 1; id <= 6; ++id)
    {
        const double rank = static_cast<double>(7 - id);
        branched2d.Add(id, Geometry::SCBox2d::FromMinMax({rank * 1e200, 0.0}, {rank * 1e200, 1.0}));
        branched3d.Add(id, Geometry::SCBox3d::FromMinMax({rank * 1e-200, 0.0, 0.0},
                                                       {rank * 1e-200, 1.0, 1.0}));
    }
    const auto nearest2d = branched2d.QueryKNearest({0.0, 0.0}, 3, 0.0);
    const auto nearest3d = branched3d.QueryKNearest({0.0, 0.0, 0.0}, 3, 0.0);
    ASSERT_EQ(nearest2d.size(), 3U);
    ASSERT_EQ(nearest3d.size(), 3U);
    for (std::size_t index = 0; index < 3; ++index)
    {
        EXPECT_EQ(nearest2d[index].id, 6U - index);
        EXPECT_EQ(nearest3d[index].id, 6U - index);
    }
}

TEST(SpatialIndexOracleTest, BoxTreeKNearestOrdersOverflowingEuclideanDistances)
{
    Geometry::SCBoxTree2d boxes2d;
    Geometry::SCBoxTree3d boxes3d;
    for (std::size_t id = 1; id <= 6; ++id)
    {
        const double x2d = -(1e308 - static_cast<double>(id - 1) * 4e306);
        const double x3d = 1.7e308 - static_cast<double>(id - 1) * 4e306;
        boxes2d.Add(id, Geometry::SCBox2d::FromMinMax({x2d, 0.0}, {x2d, 1.0}));
        boxes3d.Add(id, Geometry::SCBox3d::FromMinMax({x3d, 1e308, 0.0},
                                                      {x3d, 1e308, 1.0}));
    }
    const auto nearest2d = boxes2d.QueryKNearest({1e308, 0.0}, 3);
    const auto nearest3d = boxes3d.QueryKNearest({0.0, 0.0, 0.0}, 3);
    ASSERT_EQ(nearest2d.size(), 3U);
    ASSERT_EQ(nearest3d.size(), 3U);
    for (std::size_t index = 0; index < 3; ++index)
    {
        EXPECT_EQ(nearest2d[index].id, 6U - index);
        EXPECT_EQ(nearest3d[index].id, 6U - index);
    }
}

TEST(SpatialIndexOracleTest, ConcurrentReadQueriesKeepIndexResultsStable)
{
    Geometry::SCBoxTree2d boxes2d;
    Geometry::SCBoxTree3d boxes3d;
    Geometry::SCKDTree2d points2d;
    for (std::size_t id = 0; id < 16; ++id)
    {
        const double x = static_cast<double>(id) * 2.0;
        boxes2d.Add(id, Geometry::SCBox2d::FromMinMax({x, 0.0}, {x + 1.0, 1.0}));
        boxes3d.Add(id, Geometry::SCBox3d::FromMinMax({x, 0.0, 0.0}, {x + 1.0, 1.0, 1.0}));
        points2d.Add(id, {x, 0.0});
    }

    const auto& read2d = boxes2d;
    const auto& read3d = boxes3d;
    const auto& readPoints = points2d;
    std::atomic<bool> consistent{true};
    auto reader = [&]() {
        for (int iteration = 0; iteration < 200; ++iteration)
        {
            const auto nearest2d = read2d.QueryKNearest({0.0, 0.0}, 3, 0.0);
            const auto nearest3d = read3d.QueryKNearest({0.0, 0.0, 0.0}, 3, 0.0);
            const auto nearestPoint = readPoints.Nearest({1.9, 0.0});
            if (nearest2d.size() != 3 || nearest3d.size() != 3 || !nearestPoint ||
                nearest2d[0].id != 0 || nearest2d[1].id != 1 || nearest2d[2].id != 2 ||
                nearest3d[0].id != 0 || nearest3d[1].id != 1 || nearest3d[2].id != 2 ||
                nearestPoint->id != 1 ||
                read2d.QueryContaining({0.5, 0.5}) != std::vector<std::size_t>{0} ||
                read3d.QueryContaining({0.5, 0.5, 0.5}) != std::vector<std::size_t>{0} ||
                readPoints.QueryWithinDistance({1.9, 0.0}, 0.2) != std::vector<std::size_t>{1})
            {
                consistent.store(false, std::memory_order_relaxed);
                return;
            }
        }
    };
    std::vector<std::thread> readers;
    for (int index = 0; index < 4; ++index)
    {
        readers.emplace_back(reader);
    }
    for (auto& thread : readers)
    {
        thread.join();
    }
    EXPECT_TRUE(consistent.load(std::memory_order_relaxed));
}

TEST(SpatialIndexOracleTest, IncrementalBoxTreesKeepQueriesConsistentAfterRemovalAndMove)
{
    static_assert(std::is_const_v<std::remove_reference_t<decltype(std::declval<Geometry::SCBoxTree2d&>().Entries())>>);
    static_assert(std::is_const_v<std::remove_reference_t<decltype(std::declval<Geometry::SCBoxTree3d&>().Entries())>>);

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

TEST(SpatialIndexOracleTest, InvalidQueryPointsReturnNoHits)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();

    Geometry::SCKDTree2d points;
    points.Add(1, {0.0, 0.0});
    EXPECT_TRUE(points.Query({nan, 0.0}, 0.0).empty());
    EXPECT_TRUE(points.QueryWithinDistance({infinity, 0.0}, infinity).empty());
    EXPECT_FALSE(points.Nearest({nan, 0.0}).has_value());

    Geometry::SCBoxTree2d boxes2d;
    boxes2d.Add(1, Geometry::SCBox2d::FromMinMax({0.0, 0.0}, {1.0, 1.0}));
    EXPECT_TRUE(boxes2d.QueryContaining({nan, 0.0}).empty());
    EXPECT_TRUE(boxes2d.QueryKNearest({infinity, 0.0}, 1).empty());

    Geometry::SCBoxTree3d boxes3d;
    boxes3d.Add(1, Geometry::SCBox3d::FromMinMax({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}));
    EXPECT_TRUE(boxes3d.QueryContaining({nan, 0.0, 0.0}).empty());
    EXPECT_TRUE(boxes3d.QueryKNearest({infinity, 0.0, 0.0}, 1).empty());
}

TEST(SpatialIndexOracleTest, BoxRelationsRejectInvalidTolerance)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    const auto near2d = Geometry::SCBox2d::FromMinMax({0.0, 0.0}, {1.0, 1.0});
    const auto far2d = Geometry::SCBox2d::FromMinMax({3.0, 3.0}, {4.0, 4.0});
    const auto near3d = Geometry::SCBox3d::FromMinMax({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0});
    const auto far3d = Geometry::SCBox3d::FromMinMax({3.0, 3.0, 3.0}, {4.0, 4.0, 4.0});

    EXPECT_FALSE(Geometry::Intersects(near2d, far2d, nan));
    EXPECT_FALSE(Geometry::Intersects(near3d, far3d, nan));
    EXPECT_FALSE(Geometry::Intersects(near2d, near2d, -1.0));
    EXPECT_FALSE(Geometry::Intersects(near3d, near3d, infinity));
    EXPECT_FALSE(Geometry::Contains(near2d, Geometry::SCPoint2d{0.5, 0.5}, nan));
    EXPECT_FALSE(Geometry::Contains(near3d, Geometry::SCPoint3d{0.5, 0.5, 0.5}, infinity));
    EXPECT_FALSE(Geometry::Contains(near2d, Geometry::SCPoint2d{infinity, 0.0}, 0.0));
    EXPECT_FALSE(Geometry::Contains(near3d, Geometry::SCPoint3d{nan, 0.0, 0.0}, 0.0));
    EXPECT_TRUE(Geometry::Intersects(near2d, near2d, 0.0));
    EXPECT_TRUE(Geometry::Contains(near3d, Geometry::SCPoint3d{0.5, 0.5, 0.5}, 0.0));

    Geometry::SCBoxTree2d boxes2d;
    boxes2d.Add(1, near2d);
    EXPECT_TRUE(boxes2d.Query(far2d, nan).empty());
    Geometry::SCBoxTree3d boxes3d;
    boxes3d.Add(1, near3d);
    EXPECT_TRUE(boxes3d.Query(far3d, nan).empty());
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
