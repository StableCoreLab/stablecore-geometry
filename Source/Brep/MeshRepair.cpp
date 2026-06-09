#include "Brep/MeshRepair.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <utility>
#include <vector>

#include "Brep/MeshOps.h"
#include "Support/Geometry2d/Predicate2.h"

namespace Geometry
{
    namespace
    {
        constexpr std::size_t kNoTriangle = std::size_t(-1);

        [[nodiscard]] std::array<std::array<std::size_t, 2>, 3> TriangleEdges(const TriangleMesh::TriangleIndices& tri)
        {
            return {{
                {tri[0], tri[1]},
                {tri[1], tri[2]},
                {tri[2], tri[0]},
            }};
        }

        [[nodiscard]] bool HasSameSharedEdgeDirection(const TriangleMesh& mesh,
                                                      std::size_t firstTriangle,
                                                      std::size_t secondTriangle)
        {
            const auto firstEdges = TriangleEdges(mesh.TriangleIndicesAt(firstTriangle));
            const auto secondEdges = TriangleEdges(mesh.TriangleIndicesAt(secondTriangle));
            for (const auto& first : firstEdges)
            {
                for (const auto& second : secondEdges)
                {
                    const bool sameUndirectedEdge = (first[0] == second[0] && first[1] == second[1]) ||
                                                    (first[0] == second[1] && first[1] == second[0]);
                    if (!sameUndirectedEdge)
                    {
                        continue;
                    }

                    return first[0] == second[0] && first[1] == second[1];
                }
            }

            return false;
        }

        struct PlaneBasis
        {
            SCPoint3d origin{};
            SCVector3d u{};
            SCVector3d v{};
            SCVector3d normal{};
        };

        [[nodiscard]] double SignedArea2d(const std::vector<SCPoint2d>& points)
        {
            double area = 0.0;
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                const SCPoint2d& a = points[i];
                const SCPoint2d& b = points[(i + 1) % points.size()];
                area += a.x * b.y - b.x * a.y;
            }
            return 0.5 * area;
        }

        [[nodiscard]] bool PointInTriangle2d(
            const SCPoint2d& point, const SCPoint2d& a, const SCPoint2d& b, const SCPoint2d& c, double eps)
        {
            const double cross0 = Cross(b - a, point - a);
            const double cross1 = Cross(c - b, point - b);
            const double cross2 = Cross(a - c, point - c);
            const bool hasNegative = cross0 < -eps || cross1 < -eps || cross2 < -eps;
            const bool hasPositive = cross0 > eps || cross1 > eps || cross2 > eps;
            return !(hasNegative && hasPositive);
        }

        [[nodiscard]] bool IsEar(const std::vector<std::size_t>& polygon,
                                 std::size_t index,
                                 const std::vector<SCPoint2d>& projected,
                                 double orientationSign,
                                 double eps)
        {
            const std::size_t count = polygon.size();
            const std::size_t previous = polygon[(index + count - 1) % count];
            const std::size_t current = polygon[index];
            const std::size_t next = polygon[(index + 1) % count];
            const SCPoint2d& a = projected[previous];
            const SCPoint2d& b = projected[current];
            const SCPoint2d& c = projected[next];
            if (a.AlmostEquals(b, eps) || b.AlmostEquals(c, eps) || a.AlmostEquals(c, eps))
            {
                return false;
            }

            const double corner = Cross(b - a, c - b);
            if (orientationSign > 0.0 ? (corner <= eps) : (corner >= -eps))
            {
                return false;
            }

            for (std::size_t candidate : polygon)
            {
                if (candidate == previous || candidate == current || candidate == next)
                {
                    continue;
                }

                if (projected[candidate].AlmostEquals(a, eps) || projected[candidate].AlmostEquals(b, eps) ||
                    projected[candidate].AlmostEquals(c, eps))
                {
                    continue;
                }

                if (PointInTriangle2d(projected[candidate], a, b, c, eps))
                {
                    return false;
                }
            }

            return true;
        }

        [[nodiscard]] bool TriangulateSimplePolygon(const std::vector<SCPoint2d>& projected,
                                                    std::vector<TriangleMesh::TriangleIndices>& triangles,
                                                    double eps)
        {
            if (projected.size() < 3)
            {
                return false;
            }

            const double area = SignedArea2d(projected);
            if (std::abs(area) <= eps)
            {
                return false;
            }

            std::vector<std::size_t> polygon(projected.size());
            for (std::size_t i = 0; i < polygon.size(); ++i)
            {
                polygon[i] = i;
            }

            if (area < 0.0)
            {
                std::reverse(polygon.begin(), polygon.end());
            }

            const double orientationSign = area >= 0.0 ? 1.0 : -1.0;
            while (polygon.size() > 3)
            {
                bool clipped = false;
                for (std::size_t i = 0; i < polygon.size(); ++i)
                {
                    const std::size_t previous = polygon[(i + polygon.size() - 1) % polygon.size()];
                    const std::size_t current = polygon[i];
                    const std::size_t next = polygon[(i + 1) % polygon.size()];
                    if (projected[previous].AlmostEquals(projected[current], eps) ||
                        projected[current].AlmostEquals(projected[next], eps))
                    {
                        polygon.erase(polygon.begin() + static_cast<std::ptrdiff_t>(i));
                        clipped = true;
                        break;
                    }

                    if (!IsEar(polygon, i, projected, orientationSign, eps))
                    {
                        continue;
                    }

                    triangles.push_back(TriangleMesh::TriangleIndices{previous, current, next});
                    polygon.erase(polygon.begin() + static_cast<std::ptrdiff_t>(i));
                    clipped = true;
                    break;
                }

                if (!clipped)
                {
                    return false;
                }
            }

            triangles.push_back(TriangleMesh::TriangleIndices{polygon[0], polygon[1], polygon[2]});
            return true;
        }

        [[nodiscard]] PlaneBasis BuildPlaneBasis(const std::vector<SCPoint3d>& loopVertices, double eps)
        {
            PlaneBasis basis;
            if (loopVertices.size() < 3)
            {
                return basis;
            }

            basis.origin = loopVertices.front();
            for (std::size_t i = 1; i + 1 < loopVertices.size(); ++i)
            {
                const SCVector3d first = loopVertices[i] - basis.origin;
                const SCVector3d second = loopVertices[i + 1] - basis.origin;
                const SCVector3d normal = Cross(first, second);
                if (normal.Length() <= eps)
                {
                    continue;
                }

                basis.normal = normal.Normalized(eps);
                const SCVector3d axis =
                    std::abs(basis.normal.x) <= std::abs(basis.normal.y) &&
                            std::abs(basis.normal.x) <= std::abs(basis.normal.z)
                        ? SCVector3d{1.0, 0.0, 0.0}
                        : (std::abs(basis.normal.y) <= std::abs(basis.normal.z) ? SCVector3d{0.0, 1.0, 0.0}
                                                                                : SCVector3d{0.0, 0.0, 1.0});
                basis.u = Cross(basis.normal, axis).Normalized(eps);
                basis.v = Cross(basis.normal, basis.u).Normalized(eps);
                return basis;
            }

            return basis;
        }

        [[nodiscard]] bool IsPlanarLoop(const std::vector<SCPoint3d>& loopVertices, const PlaneBasis& basis, double eps)
        {
            if (basis.normal.Length() <= eps)
            {
                return false;
            }

            for (const SCPoint3d& vertex : loopVertices)
            {
                if (std::abs(Dot(vertex - basis.origin, basis.normal)) > eps)
                {
                    return false;
                }
            }

            return true;
        }

        [[nodiscard]] SCPoint2d ProjectToPlane(const SCPoint3d& point, const PlaneBasis& basis)
        {
            const SCVector3d delta = point - basis.origin;
            return SCPoint2d{Dot(delta, basis.u), Dot(delta, basis.v)};
        }
    }  // namespace

    TriangleMeshRepair3d OrientTriangleMeshConsistently(const TriangleMesh& mesh, double eps)
    {
        if (!mesh.IsValid(eps))
        {
            return {false, MeshRepairIssue3d::InvalidMesh, {}};
        }

        if (!IsManifoldTriangleMesh(mesh))
        {
            return {false, MeshRepairIssue3d::NonManifold, {}};
        }

        const auto adjacency = ComputeTriangleAdjacency(mesh);
        std::vector<int> flipParity(mesh.TriangleCount(), -1);
        for (std::size_t seed = 0; seed < mesh.TriangleCount(); ++seed)
        {
            if (flipParity[seed] != -1)
            {
                continue;
            }

            std::deque<std::size_t> queue{seed};
            flipParity[seed] = 0;
            while (!queue.empty())
            {
                const std::size_t triangleIndex = queue.front();
                queue.pop_front();
                for (std::size_t neighbor : adjacency[triangleIndex].adjacentTriangles)
                {
                    if (neighbor == kNoTriangle)
                    {
                        continue;
                    }

                    const bool sameDirection = HasSameSharedEdgeDirection(mesh, triangleIndex, neighbor);
                    const int expectedParity =
                        sameDirection ? 1 - flipParity[triangleIndex] : flipParity[triangleIndex];
                    if (flipParity[neighbor] == -1)
                    {
                        flipParity[neighbor] = expectedParity;
                        queue.push_back(neighbor);
                        continue;
                    }

                    if (flipParity[neighbor] != expectedParity)
                    {
                        return {false, MeshRepairIssue3d::NonOrientable, {}};
                    }
                }
            }
        }

        TriangleMesh repaired = mesh;
        auto& triangles = repaired.Triangles();
        for (std::size_t i = 0; i < triangles.size(); ++i)
        {
            if (flipParity[i] != 1)
            {
                continue;
            }

            std::swap(triangles[i][1], triangles[i][2]);
        }

        return {true, MeshRepairIssue3d::None, std::move(repaired)};
    }

    TriangleMeshRepair3d CloseSinglePlanarBoundaryLoop(const TriangleMesh& mesh, double eps)
    {
        if (!mesh.IsValid(eps))
        {
            return {false, MeshRepairIssue3d::InvalidMesh, {}};
        }

        if (!IsManifoldTriangleMesh(mesh))
        {
            return {false, MeshRepairIssue3d::NonManifold, {}};
        }

        const auto boundaryLoops = ExtractBoundaryLoops(mesh);
        if (boundaryLoops.size() != 1 || !boundaryLoops.front().closed)
        {
            return {false, MeshRepairIssue3d::UnsupportedBoundaryTopology, {}};
        }

        return ClosePlanarBoundaryLoops(mesh, eps);
    }

    TriangleMeshRepair3d ClosePlanarBoundaryLoops(const TriangleMesh& mesh, double eps)
    {
        if (!mesh.IsValid(eps))
        {
            return {false, MeshRepairIssue3d::InvalidMesh, {}};
        }

        if (!IsManifoldTriangleMesh(mesh))
        {
            return {false, MeshRepairIssue3d::NonManifold, {}};
        }

        const auto boundaryLoops = ExtractBoundaryLoops(mesh);
        if (boundaryLoops.empty())
        {
            return {false, MeshRepairIssue3d::UnsupportedBoundaryTopology, {}};
        }

        TriangleMesh repaired = mesh;
        auto& triangles = repaired.Triangles();
        for (const MeshBoundaryLoop3d& loop : boundaryLoops)
        {
            if (!loop.closed)
            {
                return {false, MeshRepairIssue3d::UnsupportedBoundaryTopology, {}};
            }

            std::vector<SCPoint3d> loopVertices;
            loopVertices.reserve(loop.vertexIndices.size());
            for (std::size_t vertexIndex : loop.vertexIndices)
            {
                loopVertices.push_back(mesh.VertexAt(vertexIndex));
            }

            const PlaneBasis basis = BuildPlaneBasis(loopVertices, eps);
            if (!IsPlanarLoop(loopVertices, basis, eps))
            {
                return {false, MeshRepairIssue3d::NonPlanarBoundary, {}};
            }

            std::vector<SCPoint2d> projected;
            projected.reserve(loopVertices.size());
            for (const SCPoint3d& vertex : loopVertices)
            {
                projected.push_back(ProjectToPlane(vertex, basis));
            }

            std::vector<TriangleMesh::TriangleIndices> capTriangles;
            capTriangles.reserve(projected.size() >= 2 ? projected.size() - 2 : 0);
            if (!TriangulateSimplePolygon(projected, capTriangles, eps))
            {
                return {false, MeshRepairIssue3d::NonPlanarBoundary, {}};
            }

            for (const TriangleMesh::TriangleIndices& tri : capTriangles)
            {
                triangles.push_back(TriangleMesh::TriangleIndices{
                    loop.vertexIndices[tri[0]], loop.vertexIndices[tri[1]], loop.vertexIndices[tri[2]]});
            }
        }

        TriangleMeshRepair3d oriented = OrientTriangleMeshConsistently(repaired, eps);
        if (!oriented.success)
        {
            return oriented;
        }

        return oriented;
    }
}  // namespace Geometry
