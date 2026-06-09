#include "Core/Measure.h"

#include <algorithm>
#include <cmath>

#include "Brep/MeshConversion.h"
#include "Brep/MeshOps.h"
#include "Core/Projection.h"
#include "Core/ShapeOps.h"

namespace Geometry
{
    double DistanceSquared(const SCPoint3d& lhs, const SCPoint3d& rhs)
    {
        return (rhs - lhs).LengthSquared();
    }

    double Distance(const SCPoint3d& lhs, const SCPoint3d& rhs)
    {
        return std::sqrt(DistanceSquared(lhs, rhs));
    }

    double DistanceSquared(const SCPoint3d& point, const SCLine3d& line, const SCGeometryTolerance3d& tolerance)
    {
        return ProjectPointToLine(point, line, tolerance).distanceSquared;
    }

    double Distance(const SCPoint3d& point, const SCLine3d& line, const SCGeometryTolerance3d& tolerance)
    {
        return std::sqrt(DistanceSquared(point, line, tolerance));
    }

    double DistanceSquared(const SCPoint3d& point, const SCPlane& plane)
    {
        const double signedDistance = plane.SignedDistanceTo(point);
        return signedDistance * signedDistance;
    }

    double Distance(const SCPoint3d& point, const SCPlane& plane)
    {
        return std::abs(plane.SignedDistanceTo(point));
    }

    double DistanceSquared(const SCPoint3d& point, const ISCCurve3d& curve, const SCGeometryTolerance3d& tolerance)
    {
        return ProjectPointToCurve(point, curve, tolerance).distanceSquared;
    }

    double Distance(const SCPoint3d& point, const ISCCurve3d& curve, const SCGeometryTolerance3d& tolerance)
    {
        return std::sqrt(DistanceSquared(point, curve, tolerance));
    }

    double DistanceSquared(const SCPoint3d& point,
                           const SCCurveOnSurface& curveOnSurface,
                           const SCGeometryTolerance3d& tolerance)
    {
        return ProjectPointToCurveOnSurface(point, curveOnSurface, tolerance).distanceSquared;
    }

    double Distance(const SCPoint3d& point, const SCCurveOnSurface& curveOnSurface, const SCGeometryTolerance3d& tolerance)
    {
        return std::sqrt(DistanceSquared(point, curveOnSurface, tolerance));
    }

    double DistanceSquared(const SCPoint3d& point, const ISCSurface& surface, const SCGeometryTolerance3d& tolerance)
    {
        return ProjectPointToSurface(point, surface, tolerance).distanceSquared;
    }

    double Distance(const SCPoint3d& point, const ISCSurface& surface, const SCGeometryTolerance3d& tolerance)
    {
        return std::sqrt(DistanceSquared(point, surface, tolerance));
    }

    double DistanceSquared(const SCPoint3d& point, const SCBrepFace& face, const SCGeometryTolerance3d& tolerance)
    {
        return ProjectPointToBrepFace(point, face, tolerance).distanceSquared;
    }

    double Distance(const SCPoint3d& point, const SCBrepFace& face, const SCGeometryTolerance3d& tolerance)
    {
        return std::sqrt(DistanceSquared(point, face, tolerance));
    }

    double DistanceSquared(const SCPoint3d& point, const SCBrepEdge& edge, const SCGeometryTolerance3d& tolerance)
    {
        return ProjectPointToBrepEdge(point, edge, tolerance).distanceSquared;
    }

    double Distance(const SCPoint3d& point, const SCBrepEdge& edge, const SCGeometryTolerance3d& tolerance)
    {
        return std::sqrt(DistanceSquared(point, edge, tolerance));
    }

    double DistanceSquared(const SCPoint3d& point, const SCBrepVertex& vertex, const SCGeometryTolerance3d& tolerance)
    {
        return ProjectPointToBrepVertex(point, vertex, tolerance).distanceSquared;
    }

    double Distance(const SCPoint3d& point, const SCBrepVertex& vertex, const SCGeometryTolerance3d& tolerance)
    {
        return std::sqrt(DistanceSquared(point, vertex, tolerance));
    }

    double DistanceSquared(const SCPoint3d& point, const SCBrepBody& body, const SCGeometryTolerance3d& tolerance)
    {
        return ProjectPointToBrepBody(point, body, tolerance).projection.distanceSquared;
    }

    double Distance(const SCPoint3d& point, const SCBrepBody& body, const SCGeometryTolerance3d& tolerance)
    {
        return std::sqrt(DistanceSquared(point, body, tolerance));
    }

    double DistanceSquared(const SCPoint3d& point, const PolyhedronFace3d& face, const SCGeometryTolerance3d& tolerance)
    {
        return ProjectPointToPolyhedronFace(point, face, tolerance).distanceSquared;
    }

    double Distance(const SCPoint3d& point, const PolyhedronFace3d& face, const SCGeometryTolerance3d& tolerance)
    {
        return std::sqrt(DistanceSquared(point, face, tolerance));
    }

    double DistanceSquared(const SCPoint3d& point, const PolyhedronBody& body, const SCGeometryTolerance3d& tolerance)
    {
        return ProjectPointToPolyhedronBody(point, body, tolerance).projection.distanceSquared;
    }

    double Distance(const SCPoint3d& point, const PolyhedronBody& body, const SCGeometryTolerance3d& tolerance)
    {
        return std::sqrt(DistanceSquared(point, body, tolerance));
    }

    double DistanceSquared(const SCPoint3d& point, const TriangleMesh& mesh, const SCGeometryTolerance3d& tolerance)
    {
        return ProjectPointToTriangleMesh(point, mesh, tolerance).distanceSquared;
    }

    double Distance(const SCPoint3d& point, const TriangleMesh& mesh, const SCGeometryTolerance3d& tolerance)
    {
        return std::sqrt(DistanceSquared(point, mesh, tolerance));
    }

    double Length(const SCLineSegment3d& segment)
    {
        return (segment.endPoint - segment.startPoint).Length();
    }

    double Length(const ISCCurve3d& curve, std::size_t sampleCount)
    {
        const SCIntervald range = curve.ParameterRange();
        if (!range.IsValid() || sampleCount == 0)
        {
            return 0.0;
        }

        const std::size_t steps = std::max<std::size_t>(sampleCount, 1);
        const double dt = range.Length() / static_cast<double>(steps);
        double total = 0.0;
        SCPoint3d previous = curve.PointAt(range.min);
        for (std::size_t i = 1; i <= steps; ++i)
        {
            const double parameter = i == steps ? range.max : range.min + dt * static_cast<double>(i);
            const SCPoint3d current = curve.PointAt(parameter);
            total += Distance(previous, current);
            previous = current;
        }
        return total;
    }

    double Length(const SCCurveOnSurface& curveOnSurface)
    {
        if (!curveOnSurface.IsValid() || curveOnSurface.PointCount() < 2)
        {
            return 0.0;
        }

        double total = 0.0;
        SCPoint3d previous = curveOnSurface.PointAt(0);
        for (std::size_t i = 1; i < curveOnSurface.PointCount(); ++i)
        {
            const SCPoint3d current = curveOnSurface.PointAt(i);
            total += Distance(previous, current);
            previous = current;
        }
        return total;
    }

    double Area(const SCTriangle3d& triangle)
    {
        return triangle.Area();
    }

    double Area(const PolyhedronFace3d& face, const SCGeometryTolerance3d& tolerance)
    {
        const SCFaceProjection3d projection = ProjectFaceToPolygon2d(face, tolerance);
        if (!projection.success)
        {
            return 0.0;
        }

        return std::abs(projection.polygon.Area());
    }

    double Area(const SCBrepFace& face, double eps)
    {
        const PolyhedronMeshConversion3d conversion = ConvertToTriangleMesh(face, eps);
        if (!conversion.success)
        {
            return 0.0;
        }

        return conversion.mesh.SurfaceArea();
    }

    double Area(const TriangleMesh& mesh)
    {
        return mesh.SurfaceArea();
    }

    double Volume(const TriangleMesh& mesh)
    {
        if (!mesh.IsValid() || !IsClosedTriangleMesh(mesh))
        {
            return 0.0;
        }

        double signedVolume = 0.0;
        for (std::size_t i = 0; i < mesh.TriangleCount(); ++i)
        {
            const SCTriangle3d triangle = mesh.TriangleAt(i);
            signedVolume += Dot(triangle.a - SCPoint3d{}, Cross(triangle.b - SCPoint3d{}, triangle.c - SCPoint3d{})) / 6.0;
        }
        return std::abs(signedVolume);
    }

    double Volume(const PolyhedronBody& body, double eps)
    {
        const PolyhedronMeshConversion3d conversion = ConvertToTriangleMesh(body, eps);
        if (!conversion.success)
        {
            return 0.0;
        }

        return Volume(conversion.mesh);
    }

    double Volume(const SCBrepBody& body, double eps)
    {
        const PolyhedronMeshConversion3d conversion = ConvertToTriangleMesh(body, eps);
        if (!conversion.success)
        {
            return 0.0;
        }

        return Volume(conversion.mesh);
    }

    SCBox3d Bounds(const ISCCurve3d& curve)
    {
        return curve.Bounds();
    }

    SCBox3d Bounds(const ISCSurface& surface)
    {
        return surface.Bounds();
    }

    SCBox3d Bounds(const SCCurveOnSurface& curveOnSurface)
    {
        return curveOnSurface.Bounds();
    }

    SCBox3d Bounds(const TriangleMesh& mesh)
    {
        return mesh.Bounds();
    }

    SCBox3d Bounds(const PolyhedronFace3d& face)
    {
        return face.Bounds();
    }

    SCBox3d Bounds(const PolyhedronBody& body)
    {
        return body.Bounds();
    }

    SCBox3d Bounds(const SCBrepVertex& vertex)
    {
        if (!vertex.IsValid())
        {
            return {};
        }

        return SCBox3d::FromMinMax(vertex.Point(), vertex.Point());
    }

    SCBox3d Bounds(const SCBrepEdge& edge)
    {
        return edge.Bounds();
    }

    SCBox3d Bounds(const SCBrepFace& face)
    {
        return face.Bounds();
    }

    SCBox3d Bounds(const SCBrepBody& body)
    {
        return body.Bounds();
    }
}  // namespace Geometry

