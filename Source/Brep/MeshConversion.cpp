#include "Brep/MeshConversion.h"

#include <algorithm>
#include <map>
#include <utility>
#include <vector>

#include "Core/Projection.h"
#include "Core/ShapeOps.h"
#include "Core/Triangulation.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "LoopGeometry.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] bool BuildPolyhedronFaceFromBrepFace(const SCBrepBody& body,
                                                           const SCBrepFace& face,
                                                           PolyhedronFace3d& polyFace,
                                                           const SCGeometryTolerance3d& tolerance)
        {
            if (!face.IsValid(tolerance))
            {
                return false;
            }

            const double eps = tolerance.distanceEpsilon;
            const auto* planeSurface = dynamic_cast<const SCPlaneSurface*>(face.SupportSurface());
            if (planeSurface == nullptr)
            {
                return false;
            }

            std::vector<SCPoint3d> outerVertices;
            if (!Detail::AppendLoopVertices(body, face.OuterLoop(), outerVertices, eps))
            {
                return false;
            }

            std::vector<PolyhedronLoop3d> holes;
            holes.reserve(face.HoleCount());
            for (std::size_t i = 0; i < face.HoleCount(); ++i)
            {
                std::vector<SCPoint3d> holeVertices;
                if (!Detail::AppendLoopVertices(body, face.HoleAt(i), holeVertices, eps))
                {
                    return false;
                }
                holes.emplace_back(std::move(holeVertices));
            }

            SCPlane actualPlane;
            if (!Detail::TryActualPlane(*planeSurface, tolerance, actualPlane)) return false;
            polyFace = PolyhedronFace3d(
                actualPlane, PolyhedronLoop3d(std::move(outerVertices)), std::move(holes));
            return polyFace.IsValid(tolerance);
        }

        [[nodiscard]] bool BuildPolygonFromBrepFaceTrims(const SCBrepFace& face,
                                                         SCPolygon2d& polygon,
                                                         const SCGeometryTolerance3d& tolerance)
        {
            if (face.OuterTrim().SupportSurface() == nullptr || !face.OuterTrim().IsValid(tolerance))
            {
                return false;
            }

            std::vector<SCPolyline2d> holes;
            holes.reserve(face.HoleTrims().size());
            for (const SCCurveOnSurface& trim : face.HoleTrims())
            {
                if (!trim.IsValid(tolerance))
                {
                    return false;
                }
                holes.push_back(trim.UvCurve());
            }

            polygon = SCPolygon2d(face.OuterTrim().UvCurve(), std::move(holes));
            return true;
        }

        [[nodiscard]] SCPoint3d LiftToSurface(const SCPoint2d& uv, const ISCSurface& surface)
        {
            return surface.PointAt(uv.x, uv.y);
        }

        class MeshVertexLookup
        {
        public:
            [[nodiscard]] std::size_t FindOrAdd(const SCPoint3d& point, std::vector<SCPoint3d>& vertices, double eps)
            {
                std::size_t best = vertices.size();
                for (auto x = byX_.lower_bound(point.x - eps);
                     x != byX_.end() && x->first <= point.x + eps; ++x)
                {
                    for (auto y = x->second.lower_bound(point.y - eps);
                         y != x->second.end() && y->first <= point.y + eps; ++y)
                    {
                        for (auto z = y->second.lower_bound(point.z - eps);
                             z != y->second.end() && z->first <= point.z + eps; ++z)
                        {
                            const std::size_t index = z->second.front();
                            if (index < best && vertices[index].AlmostEquals(point, eps))
                            {
                                best = index;
                            }
                        }
                    }
                }
                return best == vertices.size() ? Append(point, vertices) : best;
            }

            [[nodiscard]] std::size_t Append(const SCPoint3d& point, std::vector<SCPoint3d>& vertices)
            {
                const std::size_t index = vertices.size();
                vertices.push_back(point);
                byX_[point.x][point.y][point.z].push_back(index);
                return index;
            }

            void Rollback(std::vector<SCPoint3d>& vertices, std::size_t originalCount)
            {
                for (std::size_t index = vertices.size(); index > originalCount; --index)
                {
                    const SCPoint3d& point = vertices[index - 1];
                    auto x = byX_.find(point.x);
                    auto y = x->second.find(point.y);
                    auto z = y->second.find(point.z);
                    z->second.pop_back();
                    if (z->second.empty())
                    {
                        y->second.erase(z);
                    }
                    if (y->second.empty())
                    {
                        x->second.erase(y);
                    }
                    if (x->second.empty())
                    {
                        byX_.erase(x);
                    }
                }
                vertices.resize(originalCount);
            }

        private:
            using ZBuckets = std::map<double, std::vector<std::size_t>>;
            using YBuckets = std::map<double, ZBuckets>;
            std::map<double, YBuckets> byX_{};
        };

        [[nodiscard]] bool AppendFaceMesh(const TriangleMesh& faceMesh,
                                          std::vector<SCPoint3d>& vertices,
                                          std::vector<TriangleMesh::TriangleIndices>& triangles,
                                          MeshVertexLookup& vertexLookup,
                                          double eps)
        {
            if (!faceMesh.IsValid(eps))
            {
                return false;
            }

            const std::size_t originalVertexCount = vertices.size();
            const auto& faceVertices = faceMesh.Vertices();
            std::vector<std::size_t> vertexMap;
            vertexMap.reserve(faceVertices.size());
            for (const SCPoint3d& point : faceVertices)
            {
                vertexMap.push_back(vertexLookup.FindOrAdd(point, vertices, eps));
            }

            const auto& faceTriangles = faceMesh.Triangles();
            const bool weldIsValid = std::all_of(faceTriangles.begin(), faceTriangles.end(), [&](const auto& tri) {
                const std::size_t a = vertexMap[tri[0]];
                const std::size_t b = vertexMap[tri[1]];
                const std::size_t c = vertexMap[tri[2]];
                return a != b && b != c && a != c &&
                       SCTriangle3d{vertices[a], vertices[b], vertices[c]}.IsValid(eps);
            });
            if (!weldIsValid)
            {
                vertexLookup.Rollback(vertices, originalVertexCount);
                vertexMap.clear();
                for (const SCPoint3d& point : faceVertices)
                {
                    vertexMap.push_back(vertexLookup.Append(point, vertices));
                }
            }

            for (const auto& tri : faceTriangles)
            {
                triangles.push_back({vertexMap[tri[0]], vertexMap[tri[1]], vertexMap[tri[2]]});
            }
            return true;
        }

        [[nodiscard]] std::vector<SCPoint2d> FlattenBoundary(const SCPolygon2d& polygon)
        {
            std::vector<SCPoint2d> points;
            points.reserve(polygon.PointCount());
            const auto append = [&](const SCPolyline2d& ring) {
                for (std::size_t i = 0; i < ring.VertexCount(); ++i)
                {
                    points.push_back(ring.VertexAt(i));
                }
            };
            append(polygon.OuterRing());
            for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
            {
                append(polygon.HoleAt(i));
            }
            return points;
        }

        [[nodiscard]] SCPoint3d LiftToPlane(const SCPoint2d& point, const SCFaceProjection3d& projectedFace)
        {
            return projectedFace.origin + projectedFace.uAxis * point.x + projectedFace.vAxis * point.y;
        }
    }  // namespace

    PolyhedronMeshConversion3d ConvertToTriangleMesh(const PolyhedronFace3d& face, double eps)
    {
        if (!face.IsValid(eps))
        {
            return {false, MeshConversionIssue3d::InvalidFace, 0, {}};
        }

        const SCFaceProjection3d projectedFace = ProjectFaceToPolygon2d(face, SCGeometryTolerance3d{eps, eps, eps});
        if (!projectedFace.success)
        {
            return {false, MeshConversionIssue3d::TriangulationFailed, 0, {}};
        }

        const auto triangulated = Triangulate(projectedFace.polygon, eps);
        if (!triangulated)
        {
            return {false, MeshConversionIssue3d::TriangulationFailed, 0, {}};
        }

        const std::vector<SCPoint2d> contour = FlattenBoundary(projectedFace.polygon);
        std::vector<SCPoint3d> vertices3d;
        vertices3d.reserve(contour.size());
        for (const SCPoint2d& point : contour)
        {
            vertices3d.push_back(LiftToPlane(point, projectedFace));
        }

        std::vector<TriangleMesh::TriangleIndices> triangles;
        triangles.reserve(triangulated->size());
        for (const SCTriangleIndex3& triangle : *triangulated)
        {
            triangles.push_back(triangle.indices);
        }

        return {true, MeshConversionIssue3d::None, 0, TriangleMesh(vertices3d, std::move(triangles))};
    }

    PolyhedronMeshConversion3d ConvertToTriangleMesh(const PolyhedronBody& body, double eps)
    {
        if (!body.IsValid(eps))
        {
            return {false, MeshConversionIssue3d::InvalidBody, 0, {}};
        }

        std::vector<SCPoint3d> vertices;
        std::vector<TriangleMesh::TriangleIndices> triangles;
        MeshVertexLookup vertexLookup;
        for (std::size_t i = 0; i < body.FaceCount(); ++i)
        {
            const PolyhedronMeshConversion3d converted = ConvertToTriangleMesh(body.FaceAt(i), eps);
            if (!converted.success)
            {
                return {false, converted.issue, i, {}};
            }

            if (!AppendFaceMesh(converted.mesh, vertices, triangles, vertexLookup, eps))
            {
                return {false, MeshConversionIssue3d::TriangulationFailed, i, {}};
            }
        }

        return {true, MeshConversionIssue3d::None, 0, TriangleMesh(std::move(vertices), std::move(triangles))};
    }

    PolyhedronMeshConversion3d ConvertToTriangleMesh(const SCBrepFace& face,
                                                     const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !face.IsValid(tolerance))
        {
            return {false, MeshConversionIssue3d::InvalidFace, 0, {}};
        }

        const double eps = tolerance.distanceEpsilon;
        SCPolygon2d polygon{};
        if (!BuildPolygonFromBrepFaceTrims(face, polygon, tolerance))
        {
            return {false, MeshConversionIssue3d::TriangulationFailed, 0, {}};
        }

        const auto triangulated = Triangulate(polygon, eps);
        if (!triangulated)
        {
            return {false, MeshConversionIssue3d::TriangulationFailed, 0, {}};
        }

        const ISCSurface* supportSurface = face.SupportSurface();
        if (supportSurface == nullptr)
        {
            return {false, MeshConversionIssue3d::UnsupportedSurface, 0, {}};
        }

        const std::vector<SCPoint2d> contour = FlattenBoundary(polygon);
        std::vector<SCPoint3d> vertices3d;
        vertices3d.reserve(contour.size());
        for (const SCPoint2d& uv : contour)
        {
            vertices3d.push_back(LiftToSurface(uv, *supportSurface));
        }

        std::vector<TriangleMesh::TriangleIndices> triangles;
        triangles.reserve(triangulated->size());
        for (const SCTriangleIndex3& triangle : *triangulated)
        {
            triangles.push_back(triangle.indices);
        }

        return {true, MeshConversionIssue3d::None, 0, TriangleMesh(std::move(vertices3d), std::move(triangles))};
    }

    PolyhedronMeshConversion3d ConvertToTriangleMesh(const SCBrepFace& face, double eps)
    {
        return ConvertToTriangleMesh(face, SCGeometryTolerance3d{eps, eps, eps});
    }

    PolyhedronMeshConversion3d ConvertToTriangleMesh(const SCBrepBody& body,
                                                     const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !body.IsValid(tolerance))
        {
            return {false, MeshConversionIssue3d::InvalidBody, 0, {}};
        }

        const double eps = tolerance.distanceEpsilon;
        std::vector<SCPoint3d> vertices;
        std::vector<TriangleMesh::TriangleIndices> triangles;
        MeshVertexLookup vertexLookup;
        std::size_t faceIndex = 0;
        for (std::size_t shellIndex = 0; shellIndex < body.ShellCount(); ++shellIndex)
        {
            const SCBrepShell& shell = body.ShellAtRef(shellIndex);
            for (std::size_t localFaceIndex = 0; localFaceIndex < shell.FaceCount(); ++localFaceIndex, ++faceIndex)
            {
                const SCBrepFace& brepFace = shell.FaceAtRef(localFaceIndex);
                PolyhedronMeshConversion3d converted = ConvertToTriangleMesh(brepFace, tolerance);
                if (!converted.success)
                {
                    PolyhedronFace3d polyFace{};
                    if (!BuildPolyhedronFaceFromBrepFace(body, brepFace, polyFace, tolerance))
                    {
                        return {false, MeshConversionIssue3d::UnsupportedSurface, faceIndex, {}};
                    }

                    converted = ConvertToTriangleMesh(polyFace, eps);
                    if (!converted.success)
                    {
                        return {false, converted.issue, faceIndex, {}};
                    }
                }

                if (!AppendFaceMesh(converted.mesh, vertices, triangles, vertexLookup, eps))
                {
                    return {false, MeshConversionIssue3d::TriangulationFailed, faceIndex, {}};
                }
            }
        }

        return {true, MeshConversionIssue3d::None, 0, TriangleMesh(std::move(vertices), std::move(triangles))};
    }

    PolyhedronMeshConversion3d ConvertToTriangleMesh(const SCBrepBody& body, double eps)
    {
        return ConvertToTriangleMesh(body, SCGeometryTolerance3d{eps, eps, eps});
    }
}  // namespace Geometry

