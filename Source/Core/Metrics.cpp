#include "Core/Metrics.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "Core/Projection.h"
#include "../Detail/SegmentBoxRelation2d.h"

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

    namespace
    {
        // 每次公共调用只构造一次 B_eps，并逐段调用 Detail::ClassifySegmentBox。
        // 任一段分类失败时整体失败；仅在所有段成功后依据 relation 聚合。
        [[nodiscard]] bool ClassifyAllSegments(const SCBox2d& box,
                                               const SCPolyline2d& polyline,
                                               double eps,
                                               std::vector<Detail::SCBoxSegmentRelation2d>& relations)
        {
            relations.clear();
            if (!box.IsValid() || !polyline.IsValid() || !std::isfinite(eps) || eps <= 0.0)
            {
                return false;
            }
            if (polyline.SegmentCount() == 0)
            {
                // 空或零段多段线不可选择。
                return false;
            }

            // 唯一的几何容差区域为扩张闭盒 B_eps：min = box.Min - (eps,eps)，max = box.Max + (eps,eps)。
            const SCPoint2d boxMin = box.MinPoint();
            const SCPoint2d boxMax = box.MaxPoint();
            const double positiveInfinity = std::numeric_limits<double>::infinity();
            if (std::nextafter(boxMin.x, -positiveInfinity) == -positiveInfinity ||
                std::nextafter(boxMin.y, -positiveInfinity) == -positiveInfinity ||
                std::nextafter(boxMax.x, positiveInfinity) == positiveInfinity ||
                std::nextafter(boxMax.y, positiveInfinity) == positiveInfinity)
            {
                return false;
            }
            const SCPoint2d expandedMin{boxMin.x - eps, boxMin.y - eps};
            const SCPoint2d expandedMax{boxMax.x + eps, boxMax.y + eps};
            if (!expandedMin.IsValid() || !expandedMax.IsValid() || expandedMin.x > expandedMax.x ||
                expandedMin.y > expandedMax.y)
            {
                return false;
            }
            const SCBox2d expandedBox = SCBox2d::FromMinMax(expandedMin, expandedMax);
            if (!expandedBox.IsValid())
            {
                return false;
            }

            relations.reserve(polyline.SegmentCount());
            for (std::size_t i = 0; i < polyline.SegmentCount(); ++i)
            {
                const std::unique_ptr<ISCSegment2d> segment = polyline.SegmentAt(i);
                if (segment == nullptr)
                {
                    return false;
                }
                const Detail::SCBoxSegmentClassification2d classification =
                    Detail::ClassifySegmentBox(*segment, expandedBox);
                if (!classification.success)
                {
                    // 不得跳过失败段、继续聚合其余段，或将失败段当作 Disjoint。
                    return false;
                }
                relations.push_back(classification.relation);
            }
            return true;
        }

        [[nodiscard]] bool IsContainedRelation(Detail::SCBoxSegmentRelation2d relation)
        {
            return relation == Detail::SCBoxSegmentRelation2d::Contained ||
                   relation == Detail::SCBoxSegmentRelation2d::ContainedTouching;
        }

        [[nodiscard]] bool IsIntersectingRelation(Detail::SCBoxSegmentRelation2d relation)
        {
            return relation == Detail::SCBoxSegmentRelation2d::Touching ||
                   relation == Detail::SCBoxSegmentRelation2d::Contained ||
                   relation == Detail::SCBoxSegmentRelation2d::ContainedTouching ||
                   relation == Detail::SCBoxSegmentRelation2d::Crossing;
        }
    }  // namespace

    bool Contains(const SCBox2d& box, const SCPolyline2d& polyline, double eps)
    {
        std::vector<Detail::SCBoxSegmentRelation2d> relations;
        if (!ClassifyAllSegments(box, polyline, eps, relations))
        {
            return false;
        }
        for (const Detail::SCBoxSegmentRelation2d relation : relations)
        {
            if (!IsContainedRelation(relation))
            {
                return false;
            }
        }
        return true;
    }

    bool Intersects(const SCBox2d& box, const SCPolyline2d& polyline, double eps)
    {
        std::vector<Detail::SCBoxSegmentRelation2d> relations;
        if (!ClassifyAllSegments(box, polyline, eps, relations))
        {
            return false;
        }
        for (const Detail::SCBoxSegmentRelation2d relation : relations)
        {
            if (IsIntersectingRelation(relation))
            {
                return true;
            }
        }
        return false;
    }
}  // namespace Geometry
