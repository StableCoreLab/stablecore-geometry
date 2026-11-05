#include <gtest/gtest.h>

#include <vector>

#include "../../../Source/Brep/LoopGeometry.h"

namespace
{
    TEST(BrepLoopGeometry3dTest, ResolvesForwardAndReversedCoedgesFromIndependentArrays)
    {
        const std::vector<Geometry::SCBrepVertex> vertices{
            Geometry::SCBrepVertex({0, 0, 0}),
            Geometry::SCBrepVertex({1, 0, 0}),
            Geometry::SCBrepVertex({0, 1, 0})};
        const std::vector<Geometry::SCBrepEdge> edges{
            Geometry::SCBrepEdge(nullptr, 0, 1),
            Geometry::SCBrepEdge(nullptr, 2, 1),
            Geometry::SCBrepEdge(nullptr, 2, 0)};
        const Geometry::SCBrepLoop loop({{0, false}, {1, true}, {2, false}});

        std::vector<Geometry::SCPoint3d> points;
        ASSERT_TRUE(Geometry::Detail::AppendRawLoopVertices(vertices, edges, loop, points));
        ASSERT_EQ(points.size(), 3u);
        EXPECT_TRUE(points[0].AlmostEquals({0, 0, 0}));
        EXPECT_TRUE(points[1].AlmostEquals({1, 0, 0}));
        EXPECT_TRUE(points[2].AlmostEquals({0, 1, 0}));

        std::size_t start = 0;
        std::size_t end = 0;
        ASSERT_TRUE(Geometry::Detail::TryGetCoedgeVertexIndices(
            edges, vertices.size(), Geometry::SCBrepCoedge(1, true), start, end));
        EXPECT_EQ(start, 1u);
        EXPECT_EQ(end, 2u);
        EXPECT_FALSE(Geometry::Detail::TryGetCoedgeVertexIndices(
            edges, vertices.size(), Geometry::SCBrepCoedge(3), start, end));
        EXPECT_FALSE(Geometry::Detail::TryGetCoedgeVertexIndices(
            edges, 2, Geometry::SCBrepCoedge(1), start, end));
    }
}
