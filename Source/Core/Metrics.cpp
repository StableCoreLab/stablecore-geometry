#include "Core/Metrics.h"

#include <algorithm>
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

    bool Contains(const SCBox3d& box, const SCPoint3d& point, double eps)
    {
        if (!box.IsValid())
        {
            return false;
        }

        return point.x >= box.MinPoint().x - eps && point.y >= box.MinPoint().y - eps &&
               point.z >= box.MinPoint().z - eps && point.x <= box.MaxPoint().x + eps &&
               point.y <= box.MaxPoint().y + eps && point.z <= box.MaxPoint().z + eps;
    }

    bool Intersects(const SCBox3d& lhs, const SCBox3d& rhs, double eps)
    {
        if (!lhs.IsValid() || !rhs.IsValid())
        {
            return false;
        }

        return !(lhs.MaxPoint().x < rhs.MinPoint().x - eps || lhs.MaxPoint().y < rhs.MinPoint().y - eps ||
                 lhs.MaxPoint().z < rhs.MinPoint().z - eps || rhs.MaxPoint().x < lhs.MinPoint().x - eps ||
                 rhs.MaxPoint().y < lhs.MinPoint().y - eps || rhs.MaxPoint().z < lhs.MinPoint().z - eps);
    }

    double DistanceSquared(const SCPoint2d& point, const SCBox2d& box)
    {
        if (!box.IsValid())
        {
            return 0.0;
        }

        const double dx = std::max({box.MinPoint().x - point.x, 0.0, point.x - box.MaxPoint().x});
        const double dy = std::max({box.MinPoint().y - point.y, 0.0, point.y - box.MaxPoint().y});
        return dx * dx + dy * dy;
    }

    double DistanceSquared(const SCPoint3d& point, const SCBox3d& box)
    {
        if (!box.IsValid())
        {
            return 0.0;
        }

        const double dx = std::max({box.MinPoint().x - point.x, 0.0, point.x - box.MaxPoint().x});
        const double dy = std::max({box.MinPoint().y - point.y, 0.0, point.y - box.MaxPoint().y});
        const double dz = std::max({box.MinPoint().z - point.z, 0.0, point.z - box.MaxPoint().z});
        return dx * dx + dy * dy + dz * dz;
    }

    double DistanceSquared(const SCPoint3d& point, const SCLineSegment3d& segment)
    {
        const SCVector3d direction = segment.endPoint - segment.startPoint;
        const double lengthSquared = direction.LengthSquared();
        if (lengthSquared <= Geometry::kDefaultEpsilon * Geometry::kDefaultEpsilon)
        {
            return (point - segment.startPoint).LengthSquared();
        }

        const double rawParameter = Dot(point - segment.startPoint, direction) / lengthSquared;
        const double parameter = std::clamp(rawParameter, 0.0, 1.0);
        const SCPoint3d projected = segment.PointAt(parameter);
        return (point - projected).LengthSquared();
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

    SCBox2d Bounds(const SCPolyline2d& polyline)
    {
        return polyline.Bounds();
    }
}  // namespace Geometry
