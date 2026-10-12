#include "Brep/MeshConversion.h"

#include <utility>
#include <vector>

#include "Core/Projection.h"
#include "Core/ShapeOps.h"
#include "Core/Triangulation.h"
#include "Geometry3d/SCPlaneSurface.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] bool AppendLoopVerticesFromBrepLoop(const SCBrepBody& body,
                                                          const SCBrepLoop& loop,
                                                          std::vector<SCPoint3d>& vertices,
                                                          double eps)
        {
            vertices.clear();
            if (!loop.IsValid())
            {
                return false;
            }

            vertices.reserve(loop.CoedgeCount());
            for (std::size_t i = 0; i < loop.CoedgeCount(); ++i)
            {
                const SCBrepCoedge coedge = loop.CoedgeAt(i);
                if (coedge.EdgeIndex() >= body.EdgeCount())
                {
                    return false;
                }

                const SCBrepEdge edge = body.EdgeAt(coedge.EdgeIndex());
                const std::size_t vertexIndex = coedge.Reversed() ? edge.EndVertexIndex() : edge.StartVertexIndex();
                if (vertexIndex >= body.VertexCount())
                {
                    return false;
                }

                const SCPoint3d point = body.VertexAt(vertexIndex).Point();
                if (vertices.empty() || !vertices.back().AlmostEquals(point, eps))
                {
                    vertices.push_back(point);
                }
            }

            while (vertices.size() >= 2 && vertices.front().AlmostEquals(vertices.back(), eps))
            {
                vertices.pop_back();
            }

            return vertices.size() >= 3;
        }

        [[nodiscard]] bool BuildPolyhedronFaceFromBrepFace(const SCBrepBody& body,
                                                           const SCBrepFace& face,
                                                           PolyhedronFace3d& polyFace,
                                                           double eps)
        {
            if (!face.IsValid(SCGeometryTolerance3d{eps, eps, eps}))
            {
                return false;
            }

            const auto* planeSurface = dynamic_cast<const SCPlaneSurface*>(face.SupportSurface());
            if (planeSurface == nullptr)
            {
                return false;
            }

            std::vector<SCPoint3d> outerVertices;
            if (!AppendLoopVerticesFromBrepLoop(body, face.OuterLoop(), outerVertices, eps))
            {
                return false;
            }

            std::vector<PolyhedronLoop3d> holes;
            holes.reserve(face.HoleCount());
            for (std::size_t i = 0; i < face.HoleCount(); ++i)
            {
                std::vector<SCPoint3d> holeVertices;
                if (!AppendLoopVerticesFromBrepLoop(body, face.HoleAt(i), holeVertices, eps))
                {
                    return false;
                }
                holes.emplace_back(std::move(holeVertices));
            }

            polyFace = PolyhedronFace3d(
                planeSurface->SupportPlane(), PolyhedronLoop3d(std::move(outerVertices)), std::move(holes));
            return polyFace.IsValid(eps);
        }

        [[nodiscard]] bool BuildPolygonFromBrepFaceTrims(const SCBrepFace& face, SCPolygon2d& polygon)
        {
            if (face.OuterTrim().SupportSurface() == nullptr || !face.OuterTrim().IsValid())
            {
                return false;
            }

            std::vector<SCPolyline2d> holes;
            holes.reserve(face.HoleTrims().size());
            for (const SCCurveOnSurface& trim : face.HoleTrims())
            {
                if (!trim.IsValid())
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

        [[nodiscard]] std::size_t FindOrAddMeshVertex(const SCPoint3d& point, std::vector<SCPoint3d>& vertices, double eps)
        {
            for (std::size_t i = 0; i < vertices.size(); ++i)
            {
                if (vertices[i].AlmostEquals(point, eps))
                {
                    return i;
                }
            }

            vertices.push_back(point);
            return vertices.size() - 1;
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
        for (std::size_t i = 0; i < body.FaceCount(); ++i)
        {
            const PolyhedronMeshConversion3d converted = ConvertToTriangleMesh(body.FaceAt(i), eps);
            if (!converted.success)
            {
                return {false, converted.issue, i, {}};
            }

            const std::size_t offset = vertices.size();
            const auto& faceVertices = converted.mesh.Vertices();
            const auto& faceTriangles = converted.mesh.Triangles();
            vertices.insert(vertices.end(), faceVertices.begin(), faceVertices.end());
            for (const TriangleMesh::TriangleIndices& tri : faceTriangles)
            {
                triangles.push_back(TriangleMesh::TriangleIndices{tri[0] + offset, tri[1] + offset, tri[2] + offset});
            }
        }

        return {true, MeshConversionIssue3d::None, 0, TriangleMesh(std::move(vertices), std::move(triangles))};
    }

    PolyhedronMeshConversion3d ConvertToTriangleMesh(const SCBrepFace& face, double eps)
    {
        const SCGeometryTolerance3d tolerance{eps, eps, eps};
        if (!face.IsValid(tolerance))
        {
            return {false, MeshConversionIssue3d::InvalidFace, 0, {}};
        }

        SCPolygon2d polygon{};
        if (!BuildPolygonFromBrepFaceTrims(face, polygon))
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

    PolyhedronMeshConversion3d ConvertToTriangleMesh(const SCBrepBody& body, double eps)
    {
        const SCGeometryTolerance3d tolerance{eps, eps, eps};
        if (!body.IsValid(tolerance))
        {
            return {false, MeshConversionIssue3d::InvalidBody, 0, {}};
        }

        std::vector<SCPoint3d> vertices;
        std::vector<TriangleMesh::TriangleIndices> triangles;
        std::size_t faceIndex = 0;
        for (std::size_t shellIndex = 0; shellIndex < body.ShellCount(); ++shellIndex)
        {
            const SCBrepShell shell = body.ShellAt(shellIndex);
            for (std::size_t localFaceIndex = 0; localFaceIndex < shell.FaceCount(); ++localFaceIndex, ++faceIndex)
            {
                const SCBrepFace brepFace = shell.FaceAt(localFaceIndex);
                PolyhedronMeshConversion3d converted = ConvertToTriangleMesh(brepFace, eps);
                if (!converted.success)
                {
                    PolyhedronFace3d polyFace{};
                    if (!BuildPolyhedronFaceFromBrepFace(body, brepFace, polyFace, eps))
                    {
                        return {false, MeshConversionIssue3d::UnsupportedSurface, faceIndex, {}};
                    }

                    converted = ConvertToTriangleMesh(polyFace, eps);
                    if (!converted.success)
                    {
                        return {false, converted.issue, faceIndex, {}};
                    }
                }

                const auto& faceVertices = converted.mesh.Vertices();
                const auto& faceTriangles = converted.mesh.Triangles();
                std::vector<std::size_t> vertexMap;
                vertexMap.reserve(faceVertices.size());
                for (const SCPoint3d& point : faceVertices)
                {
                    vertexMap.push_back(FindOrAddMeshVertex(point, vertices, eps));
                }
                for (const TriangleMesh::TriangleIndices& tri : faceTriangles)
                {
                    triangles.push_back(
                        TriangleMesh::TriangleIndices{vertexMap[tri[0]], vertexMap[tri[1]], vertexMap[tri[2]]});
                }
            }
        }

        return {true, MeshConversionIssue3d::None, 0, TriangleMesh(std::move(vertices), std::move(triangles))};
    }
}  // namespace Geometry

