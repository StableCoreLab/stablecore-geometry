#include "sdk/GeometryRelation.h"

#include <cmath>

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
