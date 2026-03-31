#include "sdk/GeometryRelation.h"

#include <cmath>

#include "sdk/GeometryIntersection.h"
#include "sdk/GeometryProjection.h"

namespace geometry::sdk
{
PointPlaneSide3d LocatePoint(
    const Point3d& point,
    const Plane& plane,
    const GeometryTolerance3d& tolerance)
{
    if (!plane.IsValid(tolerance.distanceEpsilon))
    {
        return PointPlaneSide3d::OnPlane;
    }

    const double signedDistance = plane.SignedDistanceTo(point, tolerance.distanceEpsilon);
    if (signedDistance > tolerance.distanceEpsilon)
    {
        return PointPlaneSide3d::Above;
    }
    if (signedDistance < -tolerance.distanceEpsilon)
    {
        return PointPlaneSide3d::Below;
    }
    return PointPlaneSide3d::OnPlane;
}

PointContainment2d LocatePoint(
    const Point3d& point,
    const Curve3d& curve,
    const GeometryTolerance3d& tolerance)
{
    if (!curve.IsValid(tolerance))
    {
        return PointContainment2d::Outside;
    }

    const CurveProjection3d projection = ProjectPointToCurve(point, curve, tolerance);
    if (!projection.success)
    {
        return PointContainment2d::Outside;
    }

    return projection.distanceSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon
               ? PointContainment2d::OnBoundary
               : PointContainment2d::Outside;
}

PointContainment2d LocatePoint(
    const Point3d& point,
    const CurveOnSurface& curveOnSurface,
    const GeometryTolerance3d& tolerance)
{
    if (!curveOnSurface.IsValid(tolerance))
    {
        return PointContainment2d::Outside;
    }

    const CurveOnSurfaceProjection3d projection =
        ProjectPointToCurveOnSurface(point, curveOnSurface, tolerance);
    if (!projection.success)
    {
        return PointContainment2d::Outside;
    }

    return projection.distanceSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon
               ? PointContainment2d::OnBoundary
               : PointContainment2d::Outside;
}

PointContainment2d LocatePoint(
    const Point3d& point,
    const PolyhedronFace3d& face,
    const GeometryTolerance3d& tolerance)
{
    if (!face.IsValid(tolerance.distanceEpsilon))
    {
        return PointContainment2d::Outside;
    }

    if (LocatePoint(point, face.SupportPlane(), tolerance) != PointPlaneSide3d::OnPlane)
    {
        return PointContainment2d::Outside;
    }

    const FaceProjection3d projection = ProjectFaceToPolygon2d(face, tolerance);
    if (!projection.success)
    {
        return PointContainment2d::Outside;
    }

    const Vector3d delta = point - projection.origin;
    return LocatePoint(
        Point2d{Dot(delta, projection.uAxis), Dot(delta, projection.vAxis)},
        projection.polygon,
        tolerance.distanceEpsilon);
}

PointContainment2d LocatePoint(
    const Point3d& point,
    const PolyhedronBody& body,
    const GeometryTolerance3d& tolerance)
{
    if (!body.IsValid(tolerance.distanceEpsilon))
    {
        return PointContainment2d::Outside;
    }

    const PolyhedronBodyProjection3d projection = ProjectPointToPolyhedronBody(point, body, tolerance);
    if (projection.success &&
        projection.projection.distanceSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon)
    {
        return PointContainment2d::OnBoundary;
    }

    const Line3d probe = Line3d::FromOriginAndDirection(
        point,
        Vector3d{1.0, 0.3713906763541037, 0.52999894000318});
    const LinePolyhedronBodyIntersection3d intersections = Intersect(probe, body, tolerance);
    if (!intersections.intersects)
    {
        return PointContainment2d::Outside;
    }

    std::size_t crossingCount = 0;
    for (const LinePolyhedronFaceIntersection3d& hit : intersections.hits)
    {
        if (hit.lineParameter > tolerance.parameterEpsilon)
        {
            ++crossingCount;
        }
    }

    return (crossingCount % 2 == 1) ? PointContainment2d::Inside : PointContainment2d::Outside;
}

PointContainment2d LocatePoint(
    const Point3d& point,
    const BrepFace& face,
    const GeometryTolerance3d& tolerance)
{
    if (!face.IsValid(tolerance) || face.SupportSurface() == nullptr)
    {
        return PointContainment2d::Outside;
    }

    const BrepFaceProjection3d projection = ProjectPointToBrepFace(point, face, tolerance);
    if (!projection.success ||
        projection.distanceSquared > tolerance.distanceEpsilon * tolerance.distanceEpsilon)
    {
        return PointContainment2d::Outside;
    }

    if (projection.onBoundary)
    {
        return PointContainment2d::OnBoundary;
    }

    return projection.onTrimmedFace ? PointContainment2d::Inside : PointContainment2d::Outside;
}

PointContainment2d LocatePoint(
    const Point3d& point,
    const BrepBody& body,
    const GeometryTolerance3d& tolerance)
{
    if (!body.IsValid(tolerance))
    {
        return PointContainment2d::Outside;
    }

    const BrepBodyProjection3d projection = ProjectPointToBrepBody(point, body, tolerance);
    if (projection.success &&
        projection.projection.distanceSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon)
    {
        return PointContainment2d::OnBoundary;
    }

    const Line3d probe = Line3d::FromOriginAndDirection(
        point,
        Vector3d{1.0, 0.3713906763541037, 0.52999894000318});
    const LineBrepBodyIntersection3d intersections = Intersect(probe, body, tolerance);
    if (!intersections.intersects)
    {
        return PointContainment2d::Outside;
    }

    std::size_t crossingCount = 0;
    for (const LineBrepFaceIntersection3d& hit : intersections.hits)
    {
        if (hit.lineParameter > tolerance.parameterEpsilon)
        {
            ++crossingCount;
        }
    }

    return (crossingCount % 2 == 1) ? PointContainment2d::Inside : PointContainment2d::Outside;
}

PointContainment2d LocatePoint(
    const Point3d& point,
    const TriangleMesh& mesh,
    const GeometryTolerance3d& tolerance)
{
    if (!mesh.IsValid(tolerance.distanceEpsilon))
    {
        return PointContainment2d::Outside;
    }

    const TriangleMeshProjection3d projection = ProjectPointToTriangleMesh(point, mesh, tolerance);
    if (projection.success &&
        projection.distanceSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon)
    {
        return PointContainment2d::OnBoundary;
    }

    const Line3d probe = Line3d::FromOriginAndDirection(
        point,
        Vector3d{1.0, 0.3713906763541037, 0.52999894000318});
    const LineTriangleMeshIntersection3d intersections = Intersect(probe, mesh, tolerance);
    if (!intersections.intersects)
    {
        return PointContainment2d::Outside;
    }

    std::size_t crossingCount = 0;
    for (double parameter : intersections.lineParameters)
    {
        if (parameter > tolerance.parameterEpsilon)
        {
            ++crossingCount;
        }
    }

    return (crossingCount % 2 == 1) ? PointContainment2d::Inside : PointContainment2d::Outside;
}

bool IsParallel(
    const Vector3d& first,
    const Vector3d& second,
    const GeometryTolerance3d& tolerance)
{
    const double firstLength = first.Length();
    const double secondLength = second.Length();
    if (firstLength <= tolerance.distanceEpsilon || secondLength <= tolerance.distanceEpsilon)
    {
        return false;
    }

    const Vector3d cross = Cross(first, second);
    return cross.Length() <= tolerance.angleEpsilon * firstLength * secondLength + tolerance.distanceEpsilon;
}

bool IsPerpendicular(
    const Vector3d& first,
    const Vector3d& second,
    const GeometryTolerance3d& tolerance)
{
    const double firstLength = first.Length();
    const double secondLength = second.Length();
    if (firstLength <= tolerance.distanceEpsilon || secondLength <= tolerance.distanceEpsilon)
    {
        return false;
    }

    return std::abs(Dot(first, second)) <= tolerance.angleEpsilon * firstLength * secondLength +
           tolerance.distanceEpsilon;
}
} // namespace geometry::sdk
