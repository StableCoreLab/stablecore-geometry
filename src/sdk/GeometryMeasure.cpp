#include "sdk/GeometryMeasure.h"

#include <algorithm>
#include <cmath>

#include "sdk/GeometryMeshConversion.h"
#include "sdk/GeometryProjection.h"
#include "sdk/GeometryShapeOps.h"

namespace geometry::sdk
{
double DistanceSquared(const Point3d& lhs, const Point3d& rhs)
{
    return (rhs - lhs).LengthSquared();
}

double Distance(const Point3d& lhs, const Point3d& rhs)
{
    return std::sqrt(DistanceSquared(lhs, rhs));
}

double DistanceSquared(const Point3d& point, const Line3d& line, const GeometryTolerance3d& tolerance)
{
    return ProjectPointToLine(point, line, tolerance).distanceSquared;
}

double Distance(const Point3d& point, const Line3d& line, const GeometryTolerance3d& tolerance)
{
    return std::sqrt(DistanceSquared(point, line, tolerance));
}

double DistanceSquared(const Point3d& point, const Plane& plane)
{
    const double signedDistance = plane.SignedDistanceTo(point);
    return signedDistance * signedDistance;
}

double Distance(const Point3d& point, const Plane& plane)
{
    return std::abs(plane.SignedDistanceTo(point));
}

double DistanceSquared(const Point3d& point, const Curve3d& curve, const GeometryTolerance3d& tolerance)
{
    return ProjectPointToCurve(point, curve, tolerance).distanceSquared;
}

double Distance(const Point3d& point, const Curve3d& curve, const GeometryTolerance3d& tolerance)
{
    return std::sqrt(DistanceSquared(point, curve, tolerance));
}

double DistanceSquared(const Point3d& point, const CurveOnSurface& curveOnSurface, const GeometryTolerance3d& tolerance)
{
    return ProjectPointToCurveOnSurface(point, curveOnSurface, tolerance).distanceSquared;
}

double Distance(const Point3d& point, const CurveOnSurface& curveOnSurface, const GeometryTolerance3d& tolerance)
{
    return std::sqrt(DistanceSquared(point, curveOnSurface, tolerance));
}

double DistanceSquared(const Point3d& point, const Surface& surface, const GeometryTolerance3d& tolerance)
{
    return ProjectPointToSurface(point, surface, tolerance).distanceSquared;
}

double Distance(const Point3d& point, const Surface& surface, const GeometryTolerance3d& tolerance)
{
    return std::sqrt(DistanceSquared(point, surface, tolerance));
}

double DistanceSquared(const Point3d& point, const BrepFace& face, const GeometryTolerance3d& tolerance)
{
    return ProjectPointToBrepFace(point, face, tolerance).distanceSquared;
}

double Distance(const Point3d& point, const BrepFace& face, const GeometryTolerance3d& tolerance)
{
    return std::sqrt(DistanceSquared(point, face, tolerance));
}

double DistanceSquared(const Point3d& point, const BrepEdge& edge, const GeometryTolerance3d& tolerance)
{
    return ProjectPointToBrepEdge(point, edge, tolerance).distanceSquared;
}

double Distance(const Point3d& point, const BrepEdge& edge, const GeometryTolerance3d& tolerance)
{
    return std::sqrt(DistanceSquared(point, edge, tolerance));
}

double DistanceSquared(const Point3d& point, const BrepBody& body, const GeometryTolerance3d& tolerance)
{
    return ProjectPointToBrepBody(point, body, tolerance).projection.distanceSquared;
}

double Distance(const Point3d& point, const BrepBody& body, const GeometryTolerance3d& tolerance)
{
    return std::sqrt(DistanceSquared(point, body, tolerance));
}

double DistanceSquared(const Point3d& point, const PolyhedronFace3d& face, const GeometryTolerance3d& tolerance)
{
    return ProjectPointToPolyhedronFace(point, face, tolerance).distanceSquared;
}

double Distance(const Point3d& point, const PolyhedronFace3d& face, const GeometryTolerance3d& tolerance)
{
    return std::sqrt(DistanceSquared(point, face, tolerance));
}

double DistanceSquared(const Point3d& point, const PolyhedronBody& body, const GeometryTolerance3d& tolerance)
{
    return ProjectPointToPolyhedronBody(point, body, tolerance).projection.distanceSquared;
}

double Distance(const Point3d& point, const PolyhedronBody& body, const GeometryTolerance3d& tolerance)
{
    return std::sqrt(DistanceSquared(point, body, tolerance));
}

double Length(const LineSegment3d& segment)
{
    return (segment.endPoint - segment.startPoint).Length();
}

double Length(const Curve3d& curve, std::size_t sampleCount)
{
    const Intervald range = curve.ParameterRange();
    if (!range.IsValid() || sampleCount == 0)
    {
        return 0.0;
    }

    const std::size_t steps = std::max<std::size_t>(sampleCount, 1);
    const double dt = range.Length() / static_cast<double>(steps);
    double total = 0.0;
    Point3d previous = curve.PointAt(range.min);
    for (std::size_t i = 1; i <= steps; ++i)
    {
        const double parameter = i == steps ? range.max : range.min + dt * static_cast<double>(i);
        const Point3d current = curve.PointAt(parameter);
        total += Distance(previous, current);
        previous = current;
    }
    return total;
}

double Length(const CurveOnSurface& curveOnSurface)
{
    if (!curveOnSurface.IsValid() || curveOnSurface.PointCount() < 2)
    {
        return 0.0;
    }

    double total = 0.0;
    Point3d previous = curveOnSurface.PointAt(0);
    for (std::size_t i = 1; i < curveOnSurface.PointCount(); ++i)
    {
        const Point3d current = curveOnSurface.PointAt(i);
        total += Distance(previous, current);
        previous = current;
    }
    return total;
}

double Area(const Triangle3d& triangle)
{
    return triangle.Area();
}

double Area(const PolyhedronFace3d& face, const GeometryTolerance3d& tolerance)
{
    const FaceProjection3d projection = ProjectFaceToPolygon2d(face, tolerance);
    if (!projection.success)
    {
        return 0.0;
    }

    return std::abs(geometry::sdk::Area(projection.polygon));
}

double Area(const BrepFace& face, double eps)
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
    double signedVolume = 0.0;
    for (std::size_t i = 0; i < mesh.TriangleCount(); ++i)
    {
        const Triangle3d triangle = mesh.TriangleAt(i);
        signedVolume += Dot(triangle.a - Point3d{}, Cross(triangle.b - Point3d{}, triangle.c - Point3d{})) / 6.0;
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

double Volume(const BrepBody& body, double eps)
{
    const PolyhedronMeshConversion3d conversion = ConvertToTriangleMesh(body, eps);
    if (!conversion.success)
    {
        return 0.0;
    }

    return Volume(conversion.mesh);
}

Box3d Bounds(const Curve3d& curve)
{
    return curve.Bounds();
}

Box3d Bounds(const Surface& surface)
{
    return surface.Bounds();
}

Box3d Bounds(const CurveOnSurface& curveOnSurface)
{
    return curveOnSurface.Bounds();
}

Box3d Bounds(const TriangleMesh& mesh)
{
    return mesh.Bounds();
}

Box3d Bounds(const PolyhedronBody& body)
{
    return body.Bounds();
}

Box3d Bounds(const BrepBody& body)
{
    return body.Bounds();
}
} // namespace geometry::sdk
