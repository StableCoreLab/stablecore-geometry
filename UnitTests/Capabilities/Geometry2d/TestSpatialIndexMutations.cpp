#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "Geometry2d/SCBoxTree2d.h"
#include "Geometry2d/SCKDTree2d.h"
#include "Geometry3d/SCBoxTree3d.h"

namespace
{
    struct ModelEntry
    {
        std::size_t id;
        std::array<double, 3> lower;
        bool valid;
    };

    Geometry::SCPoint2d Point(const ModelEntry& entry)
    {
        return entry.valid ? Geometry::SCPoint2d{entry.lower[0], entry.lower[1]}
                           : Geometry::SCPoint2d{std::numeric_limits<double>::quiet_NaN(), 0.0};
    }

    Geometry::SCBox2d Box2(const ModelEntry& entry)
    {
        return entry.valid ? Geometry::SCBox2d::FromMinMax(
                                 {entry.lower[0], entry.lower[1]},
                                 {entry.lower[0] + 0.5, entry.lower[1] + 0.5})
                           : Geometry::SCBox2d{};
    }

    Geometry::SCBox3d Box3(const ModelEntry& entry)
    {
        return entry.valid ? Geometry::SCBox3d::FromMinMax(
                                 {entry.lower[0], entry.lower[1], entry.lower[2]},
                                 {entry.lower[0] + 0.5, entry.lower[1] + 0.5, entry.lower[2] + 0.5})
                           : Geometry::SCBox3d{};
    }

    std::vector<std::size_t> Sorted(std::vector<std::size_t> ids)
    {
        std::sort(ids.begin(), ids.end());
        return ids;
    }

    void CheckQueries(const Geometry::SCKDTree2d& points,
                      const Geometry::SCBoxTree2d& boxes2,
                      const Geometry::SCBoxTree3d& boxes3,
                      const std::vector<ModelEntry>& model)
    {
        EXPECT_EQ(points.Size(), model.size());
        EXPECT_EQ(boxes2.Size(), model.size());
        EXPECT_EQ(boxes3.Size(), model.size());
        const bool valid = std::all_of(model.begin(), model.end(), [](const auto& e) { return e.valid; });
        // 空集合的 IsValid 由既有容器契约另行覆盖。
        if (!model.empty())
        {
            EXPECT_EQ(points.IsValid(), valid);
            EXPECT_EQ(boxes2.IsValid(), valid);
            EXPECT_EQ(boxes3.IsValid(), valid);
        }

        for (const std::array<double, 3> q : {std::array<double, 3>{0.0, 0.0, 0.0},
                                             {11.0, -3.0, 2.0}, {255.0, 0.0, 0.0},
                                             {-23.0, 7.0, -2.0}})
        {
            SCOPED_TRACE(::testing::Message() << "query " << q[0] << ',' << q[1] << ',' << q[2]);
            std::vector<std::size_t> exact, radius, containing2, containing3, range2, range3;
            std::vector<std::pair<double, std::size_t>> nearest2, nearest3;
            double pointDistance = std::numeric_limits<double>::infinity();
            std::vector<std::size_t> nearestPointIds;
            for (const auto& entry : model)
            {
                if (!entry.valid) continue;
                double pointSquared = 0.0;
                double boxSquared = 0.0;
                bool contains = true;
                bool overlaps = true;
                for (std::size_t axis = 0; axis < 3; ++axis)
                {
                    const double delta = entry.lower[axis] - q[axis];
                    const double gap = std::max({delta, 0.0, -delta - 0.5});
                    boxSquared += gap * gap;
                    contains = contains && delta <= 0.0 && delta + 0.5 >= 0.0;
                    overlaps = overlaps && delta <= 2.0 && delta + 0.5 >= -2.0;
                    if (axis == 1)
                    {
                        nearest2.emplace_back(boxSquared, entry.id);
                        if (contains) containing2.push_back(entry.id);
                        if (overlaps) range2.push_back(entry.id);
                    }
                    if (axis < 2) pointSquared += delta * delta;
                }
                nearest3.emplace_back(boxSquared, entry.id);
                if (contains) containing3.push_back(entry.id);
                if (overlaps) range3.push_back(entry.id);
                if (pointSquared == 0.0) exact.push_back(entry.id);
                if (pointSquared <= 25.0) radius.push_back(entry.id);
                if (pointSquared < pointDistance)
                {
                    pointDistance = pointSquared;
                    nearestPointIds.clear();
                }
                if (pointSquared == pointDistance) nearestPointIds.push_back(entry.id);
            }
            const Geometry::SCPoint2d query2{q[0], q[1]};
            const Geometry::SCPoint3d query3{q[0], q[1], q[2]};
            EXPECT_EQ(Sorted(points.Query(query2, 0.0)), Sorted(exact));
            EXPECT_EQ(Sorted(points.QueryWithinDistance(query2, 5.0)), Sorted(radius));
            const auto nearest = points.Nearest(query2);
            ASSERT_EQ(nearest.has_value(), !nearestPointIds.empty());
            if (nearest)
            {
                EXPECT_DOUBLE_EQ(nearest->distanceSquared, pointDistance);
                EXPECT_NE(std::find(nearestPointIds.begin(), nearestPointIds.end(), nearest->id),
                          nearestPointIds.end());
            }
            EXPECT_EQ(Sorted(boxes2.QueryContaining(query2, 0.0)), Sorted(containing2));
            EXPECT_EQ(Sorted(boxes3.QueryContaining(query3, 0.0)), Sorted(containing3));
            EXPECT_EQ(Sorted(boxes2.Query(Geometry::SCBox2d::FromMinMax(
                          {q[0] - 2.0, q[1] - 2.0}, {q[0] + 2.0, q[1] + 2.0}), 0.0)), Sorted(range2));
            EXPECT_EQ(Sorted(boxes3.Query(Geometry::SCBox3d::FromMinMax(
                          {q[0] - 2.0, q[1] - 2.0, q[2] - 2.0},
                          {q[0] + 2.0, q[1] + 2.0, q[2] + 2.0}), 0.0)), Sorted(range3));
            std::sort(nearest2.begin(), nearest2.end());
            std::sort(nearest3.begin(), nearest3.end());
            for (std::size_t k : {0U, 1U, 7U, 300U})
            {
                const auto hits2 = boxes2.QueryKNearest(query2, k, 0.0);
                const auto hits3 = boxes3.QueryKNearest(query3, k, 0.0);
                ASSERT_EQ(hits2.size(), std::min(k, nearest2.size()));
                ASSERT_EQ(hits3.size(), std::min(k, nearest3.size()));
                for (std::size_t i = 0; i < hits2.size(); ++i)
                {
                    ASSERT_EQ(hits2[i].id, nearest2[i].second);
                    EXPECT_DOUBLE_EQ(hits2[i].distanceSquared, nearest2[i].first);
                }
                for (std::size_t i = 0; i < hits3.size(); ++i)
                {
                    ASSERT_EQ(hits3[i].id, nearest3[i].second);
                    EXPECT_DOUBLE_EQ(hits3[i].distanceSquared, nearest3[i].first);
                }
            }
        }
    }
}

TEST(SpatialIndexOracleTest, LongMixedMutationsMatchIndependentScan)
{
    Geometry::SCKDTree2d points;
    Geometry::SCBoxTree2d boxes2;
    Geometry::SCBoxTree3d boxes3;
    std::vector<ModelEntry> model;
    const auto add = [&](const ModelEntry& entry) {
        points.Add(entry.id, Point(entry));
        boxes2.Add(entry.id, Box2(entry));
        boxes3.Add(entry.id, Box3(entry));
        model.push_back(entry);
    };
    const auto update = [&](const ModelEntry& entry) {
        points.Update(entry.id, Point(entry));
        boxes2.Update(entry.id, Box2(entry));
        boxes3.Update(entry.id, Box3(entry));
        const auto found = std::find_if(model.begin(), model.end(),
                                        [&](const auto& e) { return e.id == entry.id; });
        if (found == model.end()) model.push_back(entry);
        else *found = entry;
    };
    const auto remove = [&](std::size_t id) {
        const auto count = std::erase_if(model, [id](const auto& e) { return e.id == id; });
        EXPECT_EQ(points.Remove(id), count != 0);
        EXPECT_EQ(boxes2.Remove(id), count != 0);
        EXPECT_EQ(boxes3.Remove(id), count != 0);
    };
    const auto check = [&]() {
        CheckQueries(points, boxes2, boxes3, model);
        return !::testing::Test::HasFailure();
    };
    ASSERT_TRUE(check());
    for (std::size_t i = 0; i < 256; ++i)
    {
        SCOPED_TRACE(::testing::Message() << "add " << i);
        add({i % 128, {static_cast<double>(i), 0.0, 0.0}, true});
        if (i % 32 == 31) ASSERT_TRUE(check());
    }
    for (std::size_t i = 0; i < 512; ++i)
    {
        SCOPED_TRACE(::testing::Message() << "repeat update " << i);
        update({0, {11.0, -3.0, 2.0}, i % 11 != 0});
        if (i % 32 == 31) ASSERT_TRUE(check());
    }
    for (std::size_t i = 0; i < 2048; ++i)
    {
        SCOPED_TRACE(::testing::Message() << "mixed update " << i);
        const auto signedI = static_cast<int>(i);
        update({i % 160, {static_cast<double>(signedI % 61 - 30),
                           static_cast<double>(signedI % 17 - 8),
                           static_cast<double>(signedI % 9 - 4)}, i % 11 != 0});
        if (i % 32 == 31) ASSERT_TRUE(check());
        if (i % 64 == 63)
        {
            remove((i / 64) % 160);
            ASSERT_TRUE(check());
        }
    }
    for (std::size_t id = 0; id < 160; ++id)
    {
        SCOPED_TRACE(::testing::Message() << "remove " << id);
        remove(id);
        if (id % 16 == 15) ASSERT_TRUE(check());
    }
    EXPECT_TRUE(model.empty());
    ASSERT_TRUE(check());
    add({9, {0.0, 0.0, 0.0}, true});
    points.Clear();
    boxes2.Clear();
    boxes3.Clear();
    model.clear();
    ASSERT_TRUE(check());
    update({9, {11.0, -3.0, 2.0}, true});
    ASSERT_TRUE(check());
}

TEST(SpatialIndexOracleTest, EquidistantBoxesUseIdRegardlessOfComponentDistribution)
{
    for (const double scale : {1.0, std::ldexp(1.0, 600), std::ldexp(1.0, -600)})
    {
        SCOPED_TRACE(::testing::Message() << "scale " << scale);
        Geometry::SCBoxTree3d boxes;
        boxes.Add(15, Geometry::SCBox3d::FromMinMax(
                          {23.0 * scale, -3.0 * scale, 0.0}, {23.5 * scale, -2.5 * scale, 0.5 * scale}));
        boxes.Add(52, Geometry::SCBox3d::FromMinMax(
                          {22.0 * scale, -7.0 * scale, 3.0 * scale},
                          {22.5 * scale, -6.5 * scale, 3.5 * scale}));
        // 两个盒到原点的平方距离均为 535.25 * scale²；等距时须按 ID 排序。
        const auto check = [&]() {
            const auto hits = boxes.QueryKNearest({0.0, 0.0, 0.0}, 2, 0.0);
            ASSERT_EQ(hits.size(), 2U);
            if (scale == 1.0)
            {
                EXPECT_DOUBLE_EQ(hits[0].distanceSquared, 535.25);
                EXPECT_DOUBLE_EQ(hits[1].distanceSquared, 535.25);
            }
            EXPECT_EQ(hits[0].id, 15U);
            EXPECT_EQ(hits[1].id, 52U);
            const auto first = boxes.QueryKNearest({0.0, 0.0, 0.0}, 1, 0.0);
            ASSERT_EQ(first.size(), 1U);
            EXPECT_EQ(first[0].id, 15U);
        };
        check();
        // 分开两项等距命中的树分支，复核节点顺序与剪枝也遵循相同比较规则。
        std::size_t id = 80;
        for (const double x : {-100.0, 0.0, 22.5, 100.0})
        {
            boxes.Add(id++, Geometry::SCBox3d::FromMinMax(
                                {x * scale, 100.0 * scale, 0.0},
                                {(x + 0.5) * scale, 100.5 * scale, 0.5 * scale}));
        }
        check();
    }

    // 所有坐标仍有限，但一阶距离溢出；四分之一尺度的回退也须保持等距 ID 次序。
    const double scale = std::ldexp(355.0, 1011);
    const Geometry::SCPoint3d firstPoint{23.0 * scale, 2.5 * scale, 0.0};
    const Geometry::SCPoint3d secondPoint{22.0 * scale, 6.5 * scale, 3.0 * scale};
    ASSERT_TRUE(firstPoint.IsValid());
    ASSERT_TRUE(secondPoint.IsValid());
    ASSERT_TRUE(std::isinf(std::hypot(firstPoint.x, firstPoint.y, firstPoint.z)));
    ASSERT_TRUE(std::isinf(std::hypot(secondPoint.x, secondPoint.y, secondPoint.z)));
    Geometry::SCBoxTree3d overflowing;
    overflowing.Add(15, Geometry::SCBox3d::FromMinMax(firstPoint, firstPoint));
    overflowing.Add(52, Geometry::SCBox3d::FromMinMax(secondPoint, secondPoint));
    const auto hits = overflowing.QueryKNearest({0.0, 0.0, 0.0}, 2, 0.0);
    ASSERT_EQ(hits.size(), 2U);
    EXPECT_EQ(hits[0].id, 15U);
    EXPECT_EQ(hits[1].id, 52U);
    const auto first = overflowing.QueryKNearest({0.0, 0.0, 0.0}, 1, 0.0);
    ASSERT_EQ(first.size(), 1U);
    EXPECT_EQ(first[0].id, 15U);
}
