#include <gtest/gtest.h>
#include <vector>

#include "Core/Triangulation.h"
#include "Core/Projection.h"
#include "Brep/MeshConversion.h"
#include "Brep/PolyhedronFace3d.h"
#include "../../../Source/Detail/TriangulateContours2d.h"

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

TEST(TriangulationTest, ReversedRingsKeepOriginalBoundaryIndices)
{
    const SCPolygon2d polygon(
        SCPolyline2d({{0.0, 10.0}, {10.0, 10.0}, {10.0, 0.0}, {0.0, 0.0}}, SCPolylineClosure::Closed),
        {SCPolyline2d({{4.0, 2.0}, {4.0, 4.0}, {2.0, 4.0}, {2.0, 2.0}}, SCPolylineClosure::Closed)});
    const auto triangles = Triangulate(polygon, 1e-8);
    ASSERT_TRUE(triangles.has_value());
    EXPECT_EQ(triangles->size(), 8U);
    for (const auto& triangle : *triangles)
    {
        for (std::size_t index : triangle.indices) EXPECT_LT(index, polygon.PointCount());
    }
    EXPECT_FALSE(Triangulate(polygon, -1.0).has_value());
}

TEST(TriangulationTest, ZeroToleranceKeepsStrictTriangulationAvailable)
{
    const SCPolygon2d polygon(SCPolyline2d(
        {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}}, SCPolylineClosure::Closed));
    const auto triangles = Triangulate(polygon, 0.0);
    ASSERT_TRUE(triangles.has_value());
    EXPECT_EQ(triangles->size(), 2U);
}

TEST(TriangulationTest, CallerToleranceControlsNarrowHoleClearance)
{
    const SCPolygon2d polygon(
        SCPolyline2d({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}}, SCPolylineClosure::Closed),
        {SCPolyline2d({{1e-10, 1.0}, {1e-10, 2.0}, {1.0, 2.0}, {1.0, 1.0}},
                      SCPolylineClosure::Closed)});
    EXPECT_FALSE(polygon.IsValid());
    const auto triangles = Triangulate(polygon, 1e-12);
    ASSERT_TRUE(triangles.has_value());
    EXPECT_EQ(triangles->size(), 8U);
}

TEST(TriangulationTest, CallerToleranceControlsShortValidEdges)
{
    const SCPolygon2d polygon(SCPolyline2d(
        {{0.0, 0.0}, {1e-10, 1e-10}, {0.0, 2.0}}, SCPolylineClosure::Closed));
    EXPECT_FALSE(polygon.IsValid());
    const auto triangles = Triangulate(polygon, 1e-12);
    ASSERT_TRUE(triangles.has_value());
    ASSERT_EQ(triangles->size(), 1U);
    EXPECT_EQ((*triangles)[0].indices[0], 0U);
    EXPECT_EQ((*triangles)[0].indices[1], 1U);
    EXPECT_EQ((*triangles)[0].indices[2], 2U);
    EXPECT_FALSE(Triangulate(polygon, 1e-8).has_value());
}

TEST(TriangulationTest, ShortEdgesDoNotHideSelfIntersection)
{
    const SCPolygon2d polygon(SCPolyline2d(
        {{0.0, 0.0}, {1e-10, 1e-10}, {3.0, 2.0}, {0.0, 3.0}, {2.0, 0.0}},
        SCPolylineClosure::Closed));
    EXPECT_FALSE(Triangulate(polygon, 1e-12).has_value());
}

TEST(TriangulationTest, SmallAreaWithEdgesAboveToleranceIsValid)
{
    const SCPolygon2d polygon(SCPolyline2d(
        {{0.0, 0.0}, {1e-8, 0.0}, {0.0, 1e-8}}, SCPolylineClosure::Closed));
    const auto triangles = Triangulate(polygon, 1e-12);
    ASSERT_TRUE(triangles.has_value());
    EXPECT_EQ(triangles->size(), 1U);
    EXPECT_FALSE(Triangulate(polygon, 1e-7).has_value());
}

TEST(TriangulationTest, ContourAdapterPreservesLastVertexAboveCallerTolerance)
{
    const std::vector<std::vector<SCPoint2d>> loops{
        {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 1e-10}}};
    const auto triangles = Geometry::Detail::TriangulateContours2d(loops, 1e-12);
    ASSERT_TRUE(triangles.has_value());
    ASSERT_EQ(triangles->size(), 2U);
    bool usesLastVertex = false;
    for (const auto& triangle : *triangles)
        for (std::size_t index : triangle.indices) usesLastVertex |= index == 3U;
    EXPECT_TRUE(usesLastVertex);
}

TEST(TriangulationTest, MeshConversionUsesCallerToleranceForNarrowHole)
{
    const Geometry::SCPlane plane = Geometry::SCPlane::FromPointAndNormal(
        {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
    const Geometry::PolyhedronFace3d face(plane,
        Geometry::PolyhedronLoop3d({{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0},
                                    {10.0, 10.0, 0.0}, {0.0, 10.0, 0.0}}),
        {Geometry::PolyhedronLoop3d({{1e-10, 1.0, 0.0}, {1e-10, 2.0, 0.0},
                                     {1.0, 2.0, 0.0}, {1.0, 1.0, 0.0}})});
    const Geometry::SCGeometryTolerance3d tolerance{1e-12, 1e-12, 1e-12};
    ASSERT_TRUE(face.IsValid(1e-12));
    const auto projection = Geometry::ProjectFaceToPolygon2d(face, tolerance);
    ASSERT_TRUE(projection.success);
    const auto mesh = Geometry::ConvertToTriangleMesh(face, 1e-12);
    ASSERT_TRUE(mesh.success);
    EXPECT_EQ(mesh.mesh.TriangleCount(), 8U);
}
