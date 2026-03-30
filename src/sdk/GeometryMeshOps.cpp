#include "sdk/GeometryMeshOps.h"

#include <algorithm>
#include <array>
#include <deque>
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
    std::vector<std::size_t> triangleIndices{};
    std::vector<std::size_t> edgeIndices{};
    std::vector<std::array<std::size_t, 2>> directedEdges{};
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

[[nodiscard]] const std::array<std::array<std::size_t, 2>, 3> TriangleEdges(
    const TriangleMesh::TriangleIndices& tri)
{
    return {{
        {tri[0], tri[1]},
        {tri[1], tri[2]},
        {tri[2], tri[0]},
    }};
}

[[nodiscard]] std::unordered_map<EdgeKey, EdgeRecord, EdgeKeyHash> BuildEdgeRecords(const TriangleMesh& mesh)
{
    std::unordered_map<EdgeKey, EdgeRecord, EdgeKeyHash> edgeRecords;
    edgeRecords.reserve(mesh.TriangleCount() * 3);
    for (std::size_t triangleIndex = 0; triangleIndex < mesh.TriangleCount(); ++triangleIndex)
    {
        const auto edges = TriangleEdges(mesh.TriangleIndicesAt(triangleIndex));
        for (std::size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex)
        {
            const EdgeKey key = MakeUndirectedEdgeKey(edges[edgeIndex][0], edges[edgeIndex][1]);
            EdgeRecord& record = edgeRecords[key];
            record.triangleIndices.push_back(triangleIndex);
            record.edgeIndices.push_back(edgeIndex);
            record.directedEdges.push_back(edges[edgeIndex]);
        }
    }
    return edgeRecords;
}

[[nodiscard]] bool HasOppositeOrientation(const EdgeRecord& record)
{
    if (record.directedEdges.size() != 2)
    {
        return false;
    }

    return record.directedEdges[0][0] == record.directedEdges[1][1] &&
           record.directedEdges[0][1] == record.directedEdges[1][0];
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

    const auto edgeRecords = BuildEdgeRecords(mesh);
    for (const auto& [key, record] : edgeRecords)
    {
        (void)key;
        if (record.triangleIndices.size() != 2)
        {
            continue;
        }

        const std::size_t firstTriangle = record.triangleIndices[0];
        const std::size_t secondTriangle = record.triangleIndices[1];
        const std::size_t firstEdge = record.edgeIndices[0];
        const std::size_t secondEdge = record.edgeIndices[1];
        adjacency[firstTriangle].adjacentTriangles[firstEdge] = secondTriangle;
        adjacency[secondTriangle].adjacentTriangles[secondEdge] = firstTriangle;
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

    const auto edgeRecords = BuildEdgeRecords(mesh);
    boundaryEdges.reserve(mesh.TriangleCount());
    for (const auto& [key, record] : edgeRecords)
    {
        (void)key;
        if (record.triangleIndices.size() != 1 || record.edgeIndices.size() != 1)
        {
            continue;
        }

        const std::size_t triangleIndex = record.triangleIndices[0];
        const std::size_t edgeIndex = record.edgeIndices[0];
        const auto edges = TriangleEdges(mesh.TriangleIndicesAt(triangleIndex));
        boundaryEdges.push_back(MeshBoundaryEdge3d{
            triangleIndex,
            edgeIndex,
            edges[edgeIndex]});
    }

    return boundaryEdges;
}

bool IsClosedTriangleMesh(const TriangleMesh& mesh)
{
    return mesh.IsValid() && ExtractBoundaryEdges(mesh).empty();
}

std::vector<MeshNonManifoldEdge3d> ExtractNonManifoldEdges(const TriangleMesh& mesh)
{
    std::vector<MeshNonManifoldEdge3d> nonManifoldEdges;
    if (!mesh.IsValid())
    {
        return nonManifoldEdges;
    }

    const auto edgeRecords = BuildEdgeRecords(mesh);
    for (const auto& [key, record] : edgeRecords)
    {
        if (record.triangleIndices.size() <= 2)
        {
            continue;
        }

        nonManifoldEdges.push_back(MeshNonManifoldEdge3d{
            {key.first, key.second},
            record.triangleIndices});
    }

    return nonManifoldEdges;
}

bool IsManifoldTriangleMesh(const TriangleMesh& mesh)
{
    return mesh.IsValid() && ExtractNonManifoldEdges(mesh).empty();
}

std::vector<std::vector<std::size_t>> ComputeTriangleConnectedComponents(const TriangleMesh& mesh)
{
    std::vector<std::vector<std::size_t>> components;
    if (!mesh.IsValid())
    {
        return components;
    }

    const auto edgeRecords = BuildEdgeRecords(mesh);
    std::vector<std::vector<std::size_t>> neighbors(mesh.TriangleCount());
    for (const auto& [key, record] : edgeRecords)
    {
        (void)key;
        for (std::size_t i = 0; i < record.triangleIndices.size(); ++i)
        {
            for (std::size_t j = i + 1; j < record.triangleIndices.size(); ++j)
            {
                neighbors[record.triangleIndices[i]].push_back(record.triangleIndices[j]);
                neighbors[record.triangleIndices[j]].push_back(record.triangleIndices[i]);
            }
        }
    }

    std::vector<bool> visited(mesh.TriangleCount(), false);
    for (std::size_t seed = 0; seed < mesh.TriangleCount(); ++seed)
    {
        if (visited[seed])
        {
            continue;
        }

        std::vector<std::size_t> component;
        std::deque<std::size_t> queue{seed};
        visited[seed] = true;
        while (!queue.empty())
        {
            const std::size_t triangleIndex = queue.front();
            queue.pop_front();
            component.push_back(triangleIndex);

            for (std::size_t neighbor : neighbors[triangleIndex])
            {
                if (visited[neighbor])
                {
                    continue;
                }

                visited[neighbor] = true;
                queue.push_back(neighbor);
            }
        }

        components.push_back(std::move(component));
    }

    return components;
}

bool IsConsistentlyOrientedTriangleMesh(const TriangleMesh& mesh)
{
    if (!mesh.IsValid())
    {
        return false;
    }

    const auto edgeRecords = BuildEdgeRecords(mesh);
    for (const auto& [key, record] : edgeRecords)
    {
        (void)key;
        if (record.triangleIndices.size() <= 1)
        {
            continue;
        }

        if (record.triangleIndices.size() != 2 || !HasOppositeOrientation(record))
        {
            return false;
        }
    }

    return true;
}

std::vector<MeshShell3d> ComputeMeshShells(const TriangleMesh& mesh)
{
    std::vector<MeshShell3d> shells;
    if (!mesh.IsValid())
    {
        return shells;
    }

    const auto components = ComputeTriangleConnectedComponents(mesh);
    if (components.empty())
    {
        return shells;
    }

    std::vector<std::size_t> componentByTriangle(mesh.TriangleCount(), kNoTriangle);
    shells.reserve(components.size());
    for (std::size_t componentIndex = 0; componentIndex < components.size(); ++componentIndex)
    {
        MeshShell3d shell;
        shell.triangleIndices = components[componentIndex];
        shell.closed = true;
        shell.manifold = true;
        shell.consistentlyOriented = true;
        for (std::size_t triangleIndex : components[componentIndex])
        {
            componentByTriangle[triangleIndex] = componentIndex;
        }
        shells.push_back(std::move(shell));
    }

    const auto edgeRecords = BuildEdgeRecords(mesh);
    for (const auto& [key, record] : edgeRecords)
    {
        (void)key;
        if (record.triangleIndices.empty())
        {
            continue;
        }

        const std::size_t shellIndex = componentByTriangle[record.triangleIndices.front()];
        MeshShell3d& shell = shells[shellIndex];
        if (record.triangleIndices.size() == 1)
        {
            shell.closed = false;
            continue;
        }

        if (record.triangleIndices.size() != 2)
        {
            shell.manifold = false;
            shell.consistentlyOriented = false;
            continue;
        }

        if (!HasOppositeOrientation(record))
        {
            shell.consistentlyOriented = false;
        }
    }

    return shells;
}
} // namespace geometry::sdk
