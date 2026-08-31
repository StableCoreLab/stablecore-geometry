#include <gtest/gtest.h>

#include "Core/Triangulation.h"

using Geometry::SCPoint2d;
using Geometry::SCPolygon2d;
using Geometry::SCPolyline2d;
using Geometry::SCPolylineClosure;
using Geometry::Triangulate;

namespace
{
    SCPolygon2d MakePolygonWithHole()
    {
        return SCPolygon2d(
            SCPolyline2d({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}}, SCPolylineClosure::Closed),
            {SCPolyline2d({{2.0, 2.0}, {2.0, 4.0}, {4.0, 4.0}, {4.0, 2.0}}, SCPolylineClosure::Closed)});
    }
}

TEST(TriangulationTest, TriangulatesSimplePolygonWithOriginalIndices)
{
    const SCPolygon2d polygon(SCPolyline2d({{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {2.0, 2.0}, {0.0, 4.0}},
                                           SCPolylineClosure::Closed));
    const auto result = Triangulate(polygon);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->size(), polygon.OuterRing().VertexCount() - 2);
    for (const auto& triangle : *result)
    {
        for (const std::size_t index : triangle.indices) EXPECT_LT(index, polygon.OuterRing().VertexCount());
        EXPECT_NE(triangle.indices[0], triangle.indices[1]);
        EXPECT_NE(triangle.indices[1], triangle.indices[2]);
        EXPECT_NE(triangle.indices[0], triangle.indices[2]);
    }
}

TEST(TriangulationTest, TriangulatesHoleUsingFlattenedBoundaryIndices)
{
    const SCPolygon2d polygon = MakePolygonWithHole();
    const auto result = Triangulate(polygon);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), polygon.OuterRing().VertexCount() + polygon.HoleAt(0).VertexCount());
    for (const auto& triangle : *result)
    {
        for (const std::size_t index : triangle.indices) EXPECT_LT(index, polygon.PointCount());
        EXPECT_NE(triangle.indices[0], triangle.indices[1]);
        EXPECT_NE(triangle.indices[1], triangle.indices[2]);
        EXPECT_NE(triangle.indices[0], triangle.indices[2]);
    }
}

TEST(TriangulationTest, IsDeterministic)
{
    const SCPolygon2d polygon = MakePolygonWithHole();
    const auto first = Triangulate(polygon);
    const auto second = Triangulate(polygon);
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    ASSERT_EQ(*first, *second);
}

TEST(TriangulationTest, RejectsInvalidPolygonWithoutPartialResult)
{
    EXPECT_FALSE(Triangulate(SCPolygon2d(SCPolyline2d({{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}},
                                                       SCPolylineClosure::Open)))
                     .has_value());
}
