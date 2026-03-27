#include "sdk/GeometryProjection.h"

#include <algorithm>

namespace geometry::sdk
{
LineProjection3d ProjectPointToLine(
    const Point3d& point,
    const Line3d& line,
    const GeometryTolerance3d& tolerance)
{
    if (!line.IsValid(tolerance.distanceEpsilon))
    {
        return LineProjection3d{line.origin, 0.0, (point - line.origin).LengthSquared(), false};
    }

    const double directionLengthSquared = line.direction.LengthSquared();
    const double parameter = Dot(point - line.origin, line.direction) / directionLengthSquared;
    const Point3d projectedPoint = line.PointAt(parameter);
    return LineProjection3d{projectedPoint, parameter, (point - projectedPoint).LengthSquared(), true};
}

PlaneProjection3d ProjectPointToPlane(
    const Point3d& point,
    const Plane& plane,
    const GeometryTolerance3d& tolerance)
{
    if (!plane.IsValid(tolerance.distanceEpsilon))
    {
        return PlaneProjection3d{plane.origin, 0.0, (point - plane.origin).LengthSquared()};
    }

    const Vector3d unitNormal = plane.UnitNormal(tolerance.distanceEpsilon);
    const double signedDistance = Dot(point - plane.origin, unitNormal);
    const Point3d projectedPoint = point - unitNormal * signedDistance;
    return PlaneProjection3d{projectedPoint, signedDistance, signedDistance * signedDistance};
}
} // namespace geometry::sdk
