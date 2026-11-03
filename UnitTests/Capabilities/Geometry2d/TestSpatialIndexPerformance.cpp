#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <iostream>
#include <vector>

#include "Core/Metrics.h"
#include "Core/Relation.h"
#include "Geometry2d/SCBoxTree2d.h"
#include "Geometry2d/SCKDTree2d.h"
#include "Geometry3d/SCBoxTree3d.h"

namespace
{
    using Clock = std::chrono::steady_clock;

    double MillisecondsSince(Clock::time_point start)
    {
        return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    }

    template <typename Tree, typename Value, typename Query, typename IndexedQuery, typename ScanPredicate>
    void CheckQueryMatrix(const char* name, const std::vector<Value>& values,
                          const std::vector<Query>& queries, IndexedQuery queryIndex, ScanPredicate matches)
    {
        SCOPED_TRACE(name);
        SCOPED_TRACE(values.size());
        auto start = Clock::now();
        Tree tree;
        for (std::size_t id = 0; id < values.size(); ++id) tree.Add(id, values[id]);
        const double buildMs = MillisecondsSince(start);

        std::vector<std::vector<std::size_t>> indexed(queries.size()), scanned(queries.size());
        start = Clock::now();
        for (std::size_t q = 0; q < queries.size(); ++q)
        {
            indexed[q] = queryIndex(tree, queries[q]);
            std::sort(indexed[q].begin(), indexed[q].end());
        }
        const double indexedMs = MillisecondsSince(start);
        start = Clock::now();
        for (std::size_t q = 0; q < queries.size(); ++q)
            for (std::size_t id = 0; id < values.size(); ++id)
                if (matches(values[id], queries[q])) scanned[q].push_back(id);
        const double scanMs = MillisecondsSince(start);

        std::size_t totalHits = 0;
        for (std::size_t q = 0; q < queries.size(); ++q)
        {
            SCOPED_TRACE(q);
            ASSERT_EQ(indexed[q], scanned[q]);
            totalHits += indexed[q].size();
        }
        ASSERT_GT(totalHits, 0U);
        // 只记录计时，不设置依赖机器速度的正确性门槛；断言不计入计时区间。
        std::cout << "index_benchmark name=" << name << " n=" << values.size()
                  << " queries=" << queries.size() << " build_ms=" << buildMs
                  << " indexed_ms=" << indexedMs << " scan_ms=" << scanMs
                  << " hits=" << totalHits << '\n';
    }
}

TEST(SpatialIndexPerformanceTest, KdRadiusMatrixMatchesIndependentScan)
{
    for (const std::size_t count : {256U, 2048U, 8192U})
    {
        std::vector<Geometry::SCPoint2d> points, queries;
        for (std::size_t id = 0; id < count; ++id)
            points.push_back({2.0 * (id % 128), 2.0 * (id / 128)});
        for (std::size_t q = 0; q < 128; ++q)
        {
            const auto point = points[(q * 37) % count];
            queries.push_back({point.x + 0.5, point.y + 0.5});
        }
        CheckQueryMatrix<Geometry::SCKDTree2d>("kd_radius", points, queries,
            [](const auto& tree, const auto& point) { return tree.QueryWithinDistance(point, 4.0); },
            [](const auto& point, const auto& query) { return Geometry::Distance(point, query) <= 4.0; });
    }
}

TEST(SpatialIndexPerformanceTest, Box2RangeMatrixMatchesIndependentScan)
{
    for (const std::size_t count : {256U, 2048U, 8192U})
    {
        std::vector<Geometry::SCBox2d> boxes, queries;
        for (std::size_t id = 0; id < count; ++id)
        {
            const double x = 2.0 * (id % 128), y = 2.0 * (id / 128);
            boxes.push_back(Geometry::SCBox2d::FromMinMax({x - 0.25, y - 0.25}, {x + 0.25, y + 0.25}));
        }
        for (std::size_t q = 0; q < 128; ++q)
        {
            const auto point = boxes[(q * 37) % count].Center();
            queries.push_back(Geometry::SCBox2d::FromMinMax({point.x - 3.5, point.y - 3.5},
                                                           {point.x + 4.5, point.y + 4.5}));
        }
        CheckQueryMatrix<Geometry::SCBoxTree2d>("box2_range", boxes, queries,
            [](const auto& tree, const auto& query) { return tree.Query(query, 0.0); },
            [](const auto& box, const auto& query) { return Geometry::Intersects(box, query, 0.0); });
    }
}

TEST(SpatialIndexPerformanceTest, Box3RangeMatrixMatchesIndependentScan)
{
    for (const std::size_t count : {256U, 2048U, 8192U})
    {
        std::vector<Geometry::SCBox3d> boxes, queries;
        for (std::size_t id = 0; id < count; ++id)
        {
            const double x = 2.0 * (id % 32), y = 2.0 * ((id / 32) % 16), z = 2.0 * (id / 512);
            boxes.push_back(Geometry::SCBox3d::FromMinMax({x - 0.25, y - 0.25, z - 0.25},
                                                        {x + 0.25, y + 0.25, z + 0.25}));
        }
        for (std::size_t q = 0; q < 128; ++q)
        {
            const auto point = boxes[(q * 37) % count].Center();
            queries.push_back(Geometry::SCBox3d::FromMinMax({point.x - 3.5, point.y - 3.5, point.z - 3.5},
                                                           {point.x + 4.5, point.y + 4.5, point.z + 4.5}));
        }
        CheckQueryMatrix<Geometry::SCBoxTree3d>("box3_range", boxes, queries,
            [](const auto& tree, const auto& query) { return tree.Query(query, 0.0); },
            [](const auto& box, const auto& query) { return Geometry::Intersects(box, query, 0.0); });
    }
}
