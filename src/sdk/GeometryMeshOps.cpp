#include "sdk/GeometryMeshOps.h"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <unordered_map>

namespace geometry::sdk
{
namespace
{
constexpr std::size_t kNoTriangle = std::size_t(-1);

struct EdgeRecord
{
    std::size_t triangleIndex{kNoTriangle};
    std::size_t edgeIndex{0};
};

struct EdgeKey
{
    std::size_t first{0};
    std::size_t second{0};

    [[nodiscard]] bool operator==(const EdgeKey& other) const = default;
};

struct EdgeKeyHash
{
    [[nodiscard]] std::size_t operator()(const EdgeKey& key) const
    {
        const std::size_t firstHash = std::hash<std::size_t>{}(key.first);
        const std::size_t secondHash = std::hash<std::size_t>{}(key.second);
        return firstHash ^ (secondHash + 0x9e3779b9U + (firstHash << 6U) + (firstHash >> 2U));
    }
};

[[nodiscard]] EdgeKey MakeUndirectedEdgeKey(std::size_t first, std::size_t second)
{
    return EdgeKey{
        std::min(first, second),
        std::max(first, second)};
}
} // namespace

Vector3d TriangleNormal(const TriangleMesh& mesh, std::size_t triangleIndex, double eps)
{
    if (!mesh.IsValid(eps) || triangleIndex >= mesh.TriangleCount())
    {
        return {};
    }

    const Triangle3d triangle = mesh.TriangleAt(triangleIndex);
    const Vector3d normal = triangle.Normal();
    const double length = normal.Length();
    if (length <= eps)
    {
        return {};
    }

    return normal / length;
}

std::vector<Vector3d> ComputeTriangleNormals(const TriangleMesh& mesh, double eps)
{
    std::vector<Vector3d> normals;
    normals.reserve(mesh.TriangleCount());
    for (std::size_t i = 0; i < mesh.TriangleCount(); ++i)
    {
        normals.push_back(TriangleNormal(mesh, i, eps));
    }
    return normals;
}

Vector3d VertexNormal(const TriangleMesh& mesh, std::size_t vertexIndex, double eps)
{
    if (!mesh.IsValid(eps) || vertexIndex >= mesh.VertexCount())
    {
        return {};
    }

    Vector3d accumulated{};
    for (std::size_t i = 0; i < mesh.TriangleCount(); ++i)
    {
        const TriangleMesh::TriangleIndices tri = mesh.TriangleIndicesAt(i);
        if (tri[0] != vertexIndex && tri[1] != vertexIndex && tri[2] != vertexIndex)
        {
            continue;
        }

        accumulated += mesh.TriangleAt(i).Normal();
    }

    const double length = accumulated.Length();
    if (length <= eps)
    {
        return {};
    }

    return accumulated / length;
}

std::vector<Vector3d> ComputeVertexNormals(const TriangleMesh& mesh, double eps)
{
    std::vector<Vector3d> normals;
    normals.reserve(mesh.VertexCount());
    for (std::size_t i = 0; i < mesh.VertexCount(); ++i)
    {
        normals.push_back(VertexNormal(mesh, i, eps));
    }
    return normals;
}

std::vector<MeshTriangleAdjacency3d> ComputeTriangleAdjacency(const TriangleMesh& mesh)
{
    std::vector<MeshTriangleAdjacency3d> adjacency(mesh.TriangleCount());
    if (!mesh.IsValid())
    {
        return adjacency;
    }

    std::unordered_map<EdgeKey, EdgeRecord, EdgeKeyHash> edgeOwners;
    edgeOwners.reserve(mesh.TriangleCount() * 3);
    for (std::size_t triangleIndex = 0; triangleIndex < mesh.TriangleCount(); ++triangleIndex)
    {
        const TriangleMesh::TriangleIndices tri = mesh.TriangleIndicesAt(triangleIndex);
        const std::array<std::array<std::size_t, 2>, 3> edges{{
            {tri[0], tri[1]},
            {tri[1], tri[2]},
            {tri[2], tri[0]},
        }};

        for (std::size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex)
        {
            const EdgeKey key = MakeUndirectedEdgeKey(edges[edgeIndex][0], edges[edgeIndex][1]);
            const auto [it, inserted] = edgeOwners.emplace(key, EdgeRecord{triangleIndex, edgeIndex});
            if (inserted)
            {
                continue;
            }

            adjacency[triangleIndex].adjacentTriangles[edgeIndex] = it->second.triangleIndex;
            adjacency[it->second.triangleIndex].adjacentTriangles[it->second.edgeIndex] = triangleIndex;
        }
    }

    return adjacency;
}

MeshTriangleAdjacency3d TriangleAdjacency(const TriangleMesh& mesh, std::size_t triangleIndex)
{
    if (triangleIndex >= mesh.TriangleCount())
    {
        return {};
    }

    return ComputeTriangleAdjacency(mesh).at(triangleIndex);
}

std::vector<MeshBoundaryEdge3d> ExtractBoundaryEdges(const TriangleMesh& mesh)
{
    std::vector<MeshBoundaryEdge3d> boundaryEdges;
    if (!mesh.IsValid())
    {
        return boundaryEdges;
    }

    const auto adjacency = ComputeTriangleAdjacency(mesh);
    boundaryEdges.reserve(mesh.TriangleCount());
    for (std::size_t triangleIndex = 0; triangleIndex < mesh.TriangleCount(); ++triangleIndex)
    {
        const TriangleMesh::TriangleIndices tri = mesh.TriangleIndicesAt(triangleIndex);
        const std::array<std::array<std::size_t, 2>, 3> edges{{
            {tri[0], tri[1]},
            {tri[1], tri[2]},
            {tri[2], tri[0]},
        }};

        for (std::size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex)
        {
            if (adjacency[triangleIndex].HasNeighbor(edgeIndex))
            {
                continue;
            }

            boundaryEdges.push_back(MeshBoundaryEdge3d{
                triangleIndex,
                edgeIndex,
                edges[edgeIndex]});
        }
    }

    return boundaryEdges;
}

bool IsClosedTriangleMesh(const TriangleMesh& mesh)
{
    return mesh.IsValid() && ExtractBoundaryEdges(mesh).empty();
}
} // namespace geometry::sdk
