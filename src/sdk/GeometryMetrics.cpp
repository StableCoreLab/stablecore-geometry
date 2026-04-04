#include "sdk/GeometryMetrics.h"

#include <cmath>

#include "sdk/GeometryProjection.h"

namespace geometry::sdk
{
double DistanceSquared(const Point2d& lhs, const Point2d& rhs)
{
    return (rhs - lhs).LengthSquared();
}

double Distance(const Point2d& lhs, const Point2d& rhs)
{
    return std::sqrt(DistanceSquared(lhs, rhs));
}

double DistanceSquared(const Point2d& point, const LineSegment2d& segment)
{
    return ProjectPointToLineSegment(point, segment, true).distanceSquared;
}

double DistanceSquared(const Point2d& point, const ArcSegment2d& segment)
{
    return ProjectPointToArcSegment(point, segment, true).distanceSquared;
}

double DistanceSquared(const Point2d& point, const Segment2d& segment)
{
    return ProjectPointToSegment(point, segment, true).distanceSquared;
}

double Distance(const Point2d& point, const LineSegment2d& segment)
{
    return std::sqrt(DistanceSquared(point, segment));
}

double Distance(const Point2d& point, const ArcSegment2d& segment)
{
    return std::sqrt(DistanceSquared(point, segment));
}

double Distance(const Point2d& point, const Segment2d& segment)
{
    return std::sqrt(DistanceSquared(point, segment));
}

bool Contains(const Box2d& box, const Point2d& point, double eps)
{
    if (!box.IsValid())
    {
        return false;
    }

    return point.x >= box.MinPoint().x - eps &&
           point.y >= box.MinPoint().y - eps &&
           point.x <= box.MaxPoint().x + eps &&
           point.y <= box.MaxPoint().y + eps;
}

bool Intersects(const Box2d& lhs, const Box2d& rhs, double eps)
{
    if (!lhs.IsValid() || !rhs.IsValid())
    {
        return false;
    }

    return !(lhs.MaxPoint().x < rhs.MinPoint().x - eps ||
             lhs.MaxPoint().y < rhs.MinPoint().y - eps ||
             rhs.MaxPoint().x < lhs.MinPoint().x - eps ||
             rhs.MaxPoint().y < lhs.MinPoint().y - eps);
}
} // namespace geometry::sdk

