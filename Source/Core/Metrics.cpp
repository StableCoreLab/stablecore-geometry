#include "Core/Metrics.h"

#include <cmath>

#include "Core/Projection.h"

namespace Geometry
{
    double DistanceSquared(const SCPoint2d& lhs, const SCPoint2d& rhs)
    {
        return (rhs - lhs).LengthSquared();
    }

    double Distance(const SCPoint2d& lhs, const SCPoint2d& rhs)
    {
        return std::sqrt(DistanceSquared(lhs, rhs));
    }

    bool Contains(const SCBox2d& box, const SCPoint2d& point, double eps)
    {
        if (!box.IsValid())
        {
            return false;
        }

        return point.x >= box.MinPoint().x - eps && point.y >= box.MinPoint().y - eps &&
               point.x <= box.MaxPoint().x + eps && point.y <= box.MaxPoint().y + eps;
    }

    bool Intersects(const SCBox2d& lhs, const SCBox2d& rhs, double eps)
    {
        if (!lhs.IsValid() || !rhs.IsValid())
        {
            return false;
        }

        return !(lhs.MaxPoint().x < rhs.MinPoint().x - eps || lhs.MaxPoint().y < rhs.MinPoint().y - eps ||
                 rhs.MaxPoint().x < lhs.MinPoint().x - eps || rhs.MaxPoint().y < lhs.MinPoint().y - eps);
    }

    double DistanceSquared(const SCPoint2d& point, const SCLineSegment2d& segment)
    {
        return ProjectPointToLineSegment(point, segment, true).distanceSquared;
    }

    double DistanceSquared(const SCPoint2d& point, const SCArcSegment2d& segment)
    {
        return ProjectPointToArcSegment(point, segment, true).distanceSquared;
    }

    double DistanceSquared(const SCPoint2d& point, const ISCSegment2d& segment)
    {
        return ProjectPointToSegment(point, segment, true).distanceSquared;
    }

    double Distance(const SCPoint2d& point, const SCLineSegment2d& segment)
    {
        return std::sqrt(DistanceSquared(point, segment));
    }

    double Distance(const SCPoint2d& point, const SCArcSegment2d& segment)
    {
        return std::sqrt(DistanceSquared(point, segment));
    }

    double Distance(const SCPoint2d& point, const ISCSegment2d& segment)
    {
        return std::sqrt(DistanceSquared(point, segment));
    }
}  // namespace Geometry
