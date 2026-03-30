#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "export/GeometryExport.h"
#include "sdk/TriangleMesh.h"

namespace geometry::sdk
{
struct GEOMETRY_API MeshTriangleAdjacency3d
{
    std::array<std::size_t, 3> adjacentTriangles{
        std::size_t(-1),
        std::size_t(-1),
        std::size_t(-1)};

    [[nodiscard]] bool HasNeighbor(std::size_t edgeIndex) const
    {
        return adjacentTriangles.at(edgeIndex) != std::size_t(-1);
    }
};

struct GEOMETRY_API MeshBoundaryEdge3d
{
    std::size_t triangleIndex{std::size_t(-1)};
    std::size_t edgeIndex{0};
    std::array<std::size_t, 2> vertexIndices{
        std::size_t(-1),
        std::size_t(-1)};

    [[nodiscard]] bool IsValid() const
    {
        return triangleIndex != std::size_t(-1) &&
               vertexIndices[0] != std::size_t(-1) &&
               vertexIndices[1] != std::size_t(-1);
    }
};

[[nodiscard]] GEOMETRY_API Vector3d TriangleNormal(
    const TriangleMesh& mesh,
    std::size_t triangleIndex,
    double eps = 1e-9);

[[nodiscard]] GEOMETRY_API std::vector<Vector3d> ComputeTriangleNormals(
    const TriangleMesh& mesh,
    double eps = 1e-9);

[[nodiscard]] GEOMETRY_API Vector3d VertexNormal(
    const TriangleMesh& mesh,
    std::size_t vertexIndex,
    double eps = 1e-9);

[[nodiscard]] GEOMETRY_API std::vector<Vector3d> ComputeVertexNormals(
    const TriangleMesh& mesh,
    double eps = 1e-9);

[[nodiscard]] GEOMETRY_API MeshTriangleAdjacency3d TriangleAdjacency(
    const TriangleMesh& mesh,
    std::size_t triangleIndex);

[[nodiscard]] GEOMETRY_API std::vector<MeshTriangleAdjacency3d> ComputeTriangleAdjacency(
    const TriangleMesh& mesh);

[[nodiscard]] GEOMETRY_API std::vector<MeshBoundaryEdge3d> ExtractBoundaryEdges(
    const TriangleMesh& mesh);

[[nodiscard]] GEOMETRY_API bool IsClosedTriangleMesh(const TriangleMesh& mesh);
} // namespace geometry::sdk
