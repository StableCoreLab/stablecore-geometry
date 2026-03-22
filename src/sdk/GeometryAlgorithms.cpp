#include "sdk/GeometryAlgorithms.h"

#include <algorithm>
#include <cmath>

#include "algorithm/Predicate2.h"
#include "common/Epsilon.h"
#include "sdk/GeometryConvert.h"

namespace geometry::sdk
{
namespace
{
[[nodiscard]] geometry::Point2d Interpolate(
    const geometry::Point2d& startPoint,
    const geometry::Point2d& endPoint,
    double parameter)
{
    return geometry::Point2d(
        startPoint.x + (endPoint.x - startPoint.x) * parameter,
        startPoint.y + (endPoint.y - startPoint.y) * parameter);
}
} // namespace

double DistanceSquared(const GeoPoint2d& lhs, const GeoPoint2d& rhs)
{
    return (detail::ToInternal(rhs) - detail::ToInternal(lhs)).LengthSquared();
}

double Distance(const GeoPoint2d& lhs, const GeoPoint2d& rhs)
{
    using std::sqrt;
    return sqrt(DistanceSquared(lhs, rhs));
}

ProjectionResult2d ProjectPointToSegment(
    const GeoPoint2d& point,
    const GeoPoint2d& segmentStart,
    const GeoPoint2d& segmentEnd,
    bool clampToSegment)
{
    const geometry::Point2d internalPoint = detail::ToInternal(point);
    const geometry::Point2d internalStart = detail::ToInternal(segmentStart);
    const geometry::Point2d internalEnd = detail::ToInternal(segmentEnd);
    const geometry::Vector2d segmentVector = internalEnd - internalStart;
    const double segmentLengthSquared = segmentVector.LengthSquared();

    if (geometry::IsZero(segmentLengthSquared))
    {
        return ProjectionResult2d{
            segmentStart,
            0.0,
            DistanceSquared(point, segmentStart),
            true};
    }

    const geometry::Vector2d startToPoint = internalPoint - internalStart;
    const double rawParameter = geometry::Dot(startToPoint, segmentVector) / segmentLengthSquared;
    const double parameter = clampToSegment ? std::clamp(rawParameter, 0.0, 1.0) : rawParameter;
    const geometry::Point2d projectedPoint = Interpolate(internalStart, internalEnd, parameter);

    return ProjectionResult2d{
        detail::ToSdk(projectedPoint),
        parameter,
        DistanceSquared(point, detail::ToSdk(projectedPoint)),
        clampToSegment ||
            (rawParameter >= -geometry::kDefaultEpsilon &&
             rawParameter <= 1.0 + geometry::kDefaultEpsilon)};
}

bool Contains(const GeoBox2d& box, const GeoPoint2d& point, double eps)
{
    const geometry::Box2d internalBox = detail::ToInternal(box);
    if (!internalBox.IsValid())
    {
        return false;
    }

    return point.x >= internalBox.GetMinPoint().x - eps &&
           point.y >= internalBox.GetMinPoint().y - eps &&
           point.x <= internalBox.GetMaxPoint().x + eps &&
           point.y <= internalBox.GetMaxPoint().y + eps;
}

bool Intersects(const GeoBox2d& lhs, const GeoBox2d& rhs, double eps)
{
    const geometry::Box2d left = detail::ToInternal(lhs);
    const geometry::Box2d right = detail::ToInternal(rhs);
    if (!left.IsValid() || !right.IsValid())
    {
        return false;
    }

    return !(left.GetMaxPoint().x < right.GetMinPoint().x - eps ||
             left.GetMaxPoint().y < right.GetMinPoint().y - eps ||
             right.GetMaxPoint().x < left.GetMinPoint().x - eps ||
             right.GetMaxPoint().y < left.GetMinPoint().y - eps);
}
} // namespace geometry::sdk
