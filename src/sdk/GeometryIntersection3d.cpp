#include "sdk/GeometryIntersection.h"

#include <cmath>

namespace geometry::sdk
{
LinePlaneIntersection3d Intersect(
    const Line3d& line,
    const Plane& plane,
    const GeometryTolerance3d& tolerance)
{
    LinePlaneIntersection3d result{};
    if (!line.IsValid(tolerance.distanceEpsilon) || !plane.IsValid(tolerance.distanceEpsilon))
    {
        return result;
    }

    const Vector3d unitNormal = plane.UnitNormal(tolerance.distanceEpsilon);
    const double denominator = Dot(unitNormal, line.direction);
    const double numerator = Dot(unitNormal, plane.origin - line.origin);

    if (std::abs(denominator) <= tolerance.distanceEpsilon)
    {
        result.isParallel = true;
        result.liesOnPlane = std::abs(numerator) <= tolerance.distanceEpsilon;
        result.intersects = result.liesOnPlane;
        result.point = line.origin;
        return result;
    }

    result.intersects = true;
    result.parameter = numerator / denominator;
    result.point = line.PointAt(result.parameter);
    return result;
}

PlanePlaneIntersection3d Intersect(
    const Plane& first,
    const Plane& second,
    const GeometryTolerance3d& tolerance)
{
    PlanePlaneIntersection3d result{};
    if (!first.IsValid(tolerance.distanceEpsilon) || !second.IsValid(tolerance.distanceEpsilon))
    {
        return result;
    }

    const Vector3d n1 = first.UnitNormal(tolerance.distanceEpsilon);
    const Vector3d n2 = second.UnitNormal(tolerance.distanceEpsilon);
    const Vector3d direction = Cross(n1, n2);
    const double directionLengthSquared = direction.LengthSquared();

    if (directionLengthSquared <= tolerance.distanceEpsilon * tolerance.distanceEpsilon)
    {
        result.isParallel = true;
        result.isCoincident = std::abs(second.SignedDistanceTo(first.origin, tolerance.distanceEpsilon)) <=
                              tolerance.distanceEpsilon;
        result.intersects = result.isCoincident;
        return result;
    }

    const double d1 = Dot(n1, first.origin - Point3d{});
    const double d2 = Dot(n2, second.origin - Point3d{});
    const Vector3d pointVector = Cross((d1 * n2) - (d2 * n1), direction) / directionLengthSquared;

    result.intersects = true;
    result.line = Line3d::FromOriginAndDirection(Point3d{pointVector.x, pointVector.y, pointVector.z}, direction);
    return result;
}
} // namespace geometry::sdk
