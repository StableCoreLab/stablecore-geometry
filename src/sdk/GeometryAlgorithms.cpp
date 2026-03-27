#include "sdk/GeometryAlgorithms.h"

#include <algorithm>
#include <cmath>

#include "algorithm/Predicate2.h"
#include "common/Epsilon.h"

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

double DistanceSquared(const Point2d& lhs, const Point2d& rhs)
{
    return (rhs - lhs).LengthSquared();
}

double Distance(const Point2d& lhs, const Point2d& rhs)
{
    using std::sqrt;
    return sqrt(DistanceSquared(lhs, rhs));
}

SegmentProjection2d ProjectPointToSegment(
    const Point2d& point,
    const Point2d& segmentStart,
    const Point2d& segmentEnd,
    bool clampToSegment)
{
    const geometry::Point2d internalPoint = point;
    const geometry::Point2d internalStart = segmentStart;
    const geometry::Point2d internalEnd = segmentEnd;
    const geometry::Vector2d segmentVector = internalEnd - internalStart;
    const double segmentLengthSquared = segmentVector.LengthSquared();

    if (geometry::IsZero(segmentLengthSquared))
    {
        return SegmentProjection2d{
            segmentStart,
            0.0,
            DistanceSquared(point, segmentStart),
            true};
    }

    const geometry::Vector2d startToPoint = internalPoint - internalStart;
    const double rawParameter = geometry::Dot(startToPoint, segmentVector) / segmentLengthSquared;
    const double parameter = clampToSegment ? std::clamp(rawParameter, 0.0, 1.0) : rawParameter;
    const geometry::Point2d projectedPoint = Interpolate(internalStart, internalEnd, parameter);

    return SegmentProjection2d{
        projectedPoint,
        parameter,
        DistanceSquared(point, projectedPoint),
        clampToSegment ||
            (rawParameter >= -geometry::kDefaultEpsilon &&
             rawParameter <= 1.0 + geometry::kDefaultEpsilon)};
}

bool Contains(const Box2d& box, const Point2d& point, double eps)
{
    const geometry::Box2d internalBox = box;
    if (!internalBox.IsValid())
    {
        return false;
    }

    return point.x >= internalBox.MinPoint().x - eps &&
           point.y >= internalBox.MinPoint().y - eps &&
           point.x <= internalBox.MaxPoint().x + eps &&
           point.y <= internalBox.MaxPoint().y + eps;
}

bool Intersects(const Box2d& lhs, const Box2d& rhs, double eps)
{
    const geometry::Box2d left = lhs;
    const geometry::Box2d right = rhs;
    if (!left.IsValid() || !right.IsValid())
    {
        return false;
    }

    return !(left.MaxPoint().x < right.MinPoint().x - eps ||
             left.MaxPoint().y < right.MinPoint().y - eps ||
             right.MaxPoint().x < left.MinPoint().x - eps ||
             right.MaxPoint().y < left.MinPoint().y - eps);
}
} // namespace geometry::sdk

