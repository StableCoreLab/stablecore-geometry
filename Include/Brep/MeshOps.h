#pragma once
#include <array>
#include <vector>
#include "Export/GeometryExport.h"
#include "Geometry3d/TriangleMesh.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    struct GEOMETRY_API MeshTriangleAdjacency3d
    {
        std::array<std::size_t, 3> adjacentTriangles{std::size_t(-1), std::size_t(-1), std::size_t(-1)};

        [[nodiscard]] bool HasNeighbor(std::size_t edgeIndex) const
        {
            return adjacentTriangles.at(edgeIndex) != std::size_t(-1);
        }
    };

    struct GEOMETRY_API MeshBoundaryEdge3d
    {
        std::size_t triangleIndex{std::size_t(-1)};
        std::size_t edgeIndex{0};
        std::array<std::size_t, 2> vertexIndices{std::size_t(-1), std::size_t(-1)};

        [[nodiscard]] bool IsValid() const
        {
            return triangleIndex != std::size_t(-1) && vertexIndices[0] != std::size_t(-1) &&
                   vertexIndices[1] != std::size_t(-1);
        }
    };

    struct GEOMETRY_API MeshBoundaryLoop3d
    {
        std::vector<std::size_t> vertexIndices{};
        bool closed{false};

        [[nodiscard]] bool IsValid() const
        {
            return vertexIndices.size() >= (closed ? 3U : 2U);
        }
    };

    struct GEOMETRY_API MeshNonManifoldEdge3d
    {
        std::array<std::size_t, 2> vertexIndices{std::size_t(-1), std::size_t(-1)};
        std::vector<std::size_t> incidentTriangles{};

        [[nodiscard]] bool IsValid() const
        {
            return vertexIndices[0] != std::size_t(-1) && vertexIndices[1] != std::size_t(-1) &&
                   incidentTriangles.size() > 2;
        }
    };

    struct GEOMETRY_API MeshShell3d
    {
        std::vector<std::size_t> triangleIndices{};
        bool closed{false};
        bool manifold{false};
        bool consistentlyOriented{false};

        [[nodiscard]] bool IsValid() const
        {
            return !triangleIndices.empty();
        }
    };

    [[nodiscard]] GEOMETRY_API SCVector3d TriangleNormal(const TriangleMesh& mesh,
                                                       std::size_t triangleIndex,
                                                       double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API std::vector<SCVector3d> ComputeTriangleNormals(const TriangleMesh& mesh,
                                                                            double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCVector3d VertexNormal(const TriangleMesh& mesh,
                                                     std::size_t vertexIndex,
                                                     double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API std::vector<SCVector3d> ComputeVertexNormals(const TriangleMesh& mesh,
                                                                          double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API MeshTriangleAdjacency3d TriangleAdjacency(const TriangleMesh& mesh,
                                                                         std::size_t triangleIndex);

    [[nodiscard]] GEOMETRY_API std::vector<MeshTriangleAdjacency3d> ComputeTriangleAdjacency(const TriangleMesh& mesh);

    [[nodiscard]] GEOMETRY_API std::vector<MeshBoundaryEdge3d> ExtractBoundaryEdges(const TriangleMesh& mesh);

    [[nodiscard]] GEOMETRY_API std::vector<MeshBoundaryLoop3d> ExtractBoundaryLoops(const TriangleMesh& mesh);

    [[nodiscard]] GEOMETRY_API bool IsClosedTriangleMesh(const TriangleMesh& mesh);

    [[nodiscard]] GEOMETRY_API std::vector<MeshNonManifoldEdge3d> ExtractNonManifoldEdges(const TriangleMesh& mesh);

    [[nodiscard]] GEOMETRY_API bool IsManifoldTriangleMesh(const TriangleMesh& mesh);

    [[nodiscard]] GEOMETRY_API std::vector<std::vector<std::size_t>> ComputeTriangleConnectedComponents(
        const TriangleMesh& mesh);

    [[nodiscard]] GEOMETRY_API bool IsConsistentlyOrientedTriangleMesh(const TriangleMesh& mesh);

    [[nodiscard]] GEOMETRY_API std::vector<MeshShell3d> ComputeMeshShells(const TriangleMesh& mesh);
}  // namespace Geometry
