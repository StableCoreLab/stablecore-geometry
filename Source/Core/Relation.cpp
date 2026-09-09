#include "Core/Relation.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "Core/Intersection.h"
#include "Core/Projection.h"
#include "Core/ShapeOps.h"
#include "Support/Epsilon.h"
#include "Support/Geometry2d/Predicate2.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] bool IsPointOnLineSegment(const SCPoint2d& point, const SCLineSegment2d& segment, double eps)
        {
            const auto projection = ProjectPointToLineSegment(point, segment, true);
            return projection.distanceSquared <= eps * eps;
        }

        [[nodiscard]] bool IsPointOnArcSegment(const SCPoint2d& point, const SCArcSegment2d& segment, double eps)
        {
            const auto projection = ProjectPointToArcSegment(point, segment, true);
            return projection.distanceSquared <= eps * eps;
        }

        [[nodiscard]] std::size_t CountLineSegmentRayCrossings(const SCLineSegment2d& segment,
                                                               const SCPoint2d& point,
                                                               double eps)
        {
            const double startY = segment.startPoint.y;
            const double endY = segment.endPoint.y;
            const bool straddles = (startY <= point.y && endY > point.y) || (endY <= point.y && startY > point.y);
            if (!straddles)
            {
                return 0;
            }

            const double x = segment.startPoint.x +
                             (segment.endPoint.x - segment.startPoint.x) * (point.y - startY) / (endY - startY);
            return x > point.x + eps ? 1U : 0U;
        }

        [[nodiscard]] std::size_t CountArcSegmentRayCrossings(const SCArcSegment2d& segment,
                                                              const SCPoint2d& point,
                                                              double eps)
        {
            const double dy = point.y - segment.center.y;
            if (std::abs(dy) >= segment.radius - eps)
            {
                return 0;
            }

            const double radialSquared = segment.radius * segment.radius - dy * dy;
            if (radialSquared <= 0.0)
            {
                return 0;
            }

            const double dx = std::sqrt(radialSquared);
            const double candidateX[2] = {segment.center.x - dx, segment.center.x + dx};
            std::size_t count = 0;
            for (const double x : candidateX)
            {
                if (x <= point.x + eps)
                {
                    continue;
                }

                const SCPoint2d candidate{x, point.y};
                const SCSegmentProjection2d projection = ProjectPointToArcSegment(candidate, segment, false);
                if (projection.distanceSquared > eps * eps || !projection.isOnSegment)
                {
                    continue;
                }

                if (projection.parameter < -eps || projection.parameter >= 1.0 - eps)
                {
                    continue;
                }

                ++count;
            }

            return count;
        }

        [[nodiscard]] std::size_t CountSegmentRayCrossings(const ISCSegment2d& segment, const SCPoint2d& point, double eps)
        {
            if (segment.Kind() == SCSegmentKind2::Line)
            {
                return CountLineSegmentRayCrossings(static_cast<const SCLineSegment2d&>(segment), point, eps);
            }

            return CountArcSegmentRayCrossings(static_cast<const SCArcSegment2d&>(segment), point, eps);
        }

        [[nodiscard]] SCPointContainment2d LocatePointInRing(const SCPoint2d& point, const SCPolyline2d& ring, double eps)
        {
            if (!ring.IsClosed() || !ring.IsValid() || ring.SegmentCount() == 0)
            {
                return SCPointContainment2d::Outside;
            }

            std::size_t crossingCount = 0;
            for (std::size_t i = 0; i < ring.SegmentCount(); ++i)
            {
                std::unique_ptr<ISCSegment2d> segment = ring.SegmentAt(i);
                if (segment == nullptr)
                {
                    continue;
                }

                if (LocatePoint(point, *segment, eps) == SCPointContainment2d::OnBoundary)
                {
                    return SCPointContainment2d::OnBoundary;
                }

                crossingCount += CountSegmentRayCrossings(*segment, point, eps);
            }

            return (crossingCount % 2U) == 1U ? SCPointContainment2d::Inside : SCPointContainment2d::Outside;
        }
    }  // namespace

    SCPointContainment2d LocatePoint(const SCPoint2d& point, const SCLineSegment2d& segment, double eps)
    {
        if (!segment.IsValid())
        {
            return SCPointContainment2d::Outside;
        }

        if (IsPointOnLineSegment(point, segment, eps))
        {
            return SCPointContainment2d::OnBoundary;
        }

        return SCPointContainment2d::Outside;
    }

    SCPointContainment2d LocatePoint(const SCPoint2d& point, const SCArcSegment2d& segment, double eps)
    {
        if (!segment.IsValid())
        {
            return SCPointContainment2d::Outside;
        }

        if (IsPointOnArcSegment(point, segment, eps))
        {
            return SCPointContainment2d::OnBoundary;
        }

        return SCPointContainment2d::Outside;
    }

    SCPointContainment2d LocatePoint(const SCPoint2d& point, const ISCSegment2d& segment, double eps)
    {
        if (segment.Kind() == Geometry::SCSegmentKind2::Line)
        {
            return LocatePoint(point, static_cast<const SCLineSegment2d&>(segment), eps);
        }
        if (segment.Kind() == Geometry::SCSegmentKind2::Arc)
        {
            return LocatePoint(point, static_cast<const SCArcSegment2d&>(segment), eps);
        }
        return SCPointContainment2d::Outside;
    }

    SCPointContainment2d LocatePoint(const SCPoint2d& point, const SCPolyline2d& polyline, double eps)
    {
        if (!polyline.IsValid())
        {
            return SCPointContainment2d::Outside;
        }

        if (!polyline.IsClosed())
        {
            for (std::size_t i = 0; i < polyline.SegmentCount(); ++i)
            {
                std::unique_ptr<ISCSegment2d> segment = polyline.SegmentAt(i);
                if (segment != nullptr && LocatePoint(point, *segment, eps) == SCPointContainment2d::OnBoundary)
                {
                    return SCPointContainment2d::OnBoundary;
                }
            }
            return SCPointContainment2d::Outside;
        }

        return LocatePointInRing(point, polyline, eps);
    }

    SCPointContainment2d LocatePoint(const SCPoint2d& point, const SCPolygon2d& polygon, double eps)
    {
        if (!polygon.IsValid())
        {
            return SCPointContainment2d::Outside;
        }

        const SCPointContainment2d outer = LocatePointInRing(point, polygon.OuterRing(), eps);
        if (outer == SCPointContainment2d::Outside)
        {
            return SCPointContainment2d::Outside;
        }
        if (outer == SCPointContainment2d::OnBoundary)
        {
            return SCPointContainment2d::OnBoundary;
        }

        for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
        {
            const SCPointContainment2d hole = LocatePointInRing(point, polygon.HoleAt(i), eps);
            if (hole == SCPointContainment2d::OnBoundary)
            {
                return SCPointContainment2d::OnBoundary;
            }
            if (hole == SCPointContainment2d::Inside)
            {
                return SCPointContainment2d::Outside;
            }
        }

        return SCPointContainment2d::Inside;
    }

    bool IsParallel(const SCLineSegment2d& first, const SCLineSegment2d& second, double eps)
    {
        if (!first.IsValid() || !second.IsValid())
        {
            return false;
        }

        return std::abs(Cross(first.endPoint - first.startPoint, second.endPoint - second.startPoint)) <= eps;
    }

    bool IsAntiParallel(const SCLineSegment2d& first, const SCLineSegment2d& second, double eps)
    {
        if (!IsParallel(first, second, eps))
        {
            return false;
        }

        return Dot(first.endPoint - first.startPoint, second.endPoint - second.startPoint) < 0.0;
    }

    bool IsSameDirection(const SCLineSegment2d& first, const SCLineSegment2d& second, double eps)
    {
        if (!IsParallel(first, second, eps))
        {
            return false;
        }

        return Dot(first.endPoint - first.startPoint, second.endPoint - second.startPoint) > 0.0;
    }

    bool IsEqual(const SCLineSegment2d& first, const SCLineSegment2d& second, bool ignoreDirection, double eps)
    {
        if (first.startPoint.AlmostEquals(second.startPoint, eps) && first.endPoint.AlmostEquals(second.endPoint, eps))
        {
            return true;
        }

        return ignoreDirection && first.startPoint.AlmostEquals(second.endPoint, eps) &&
               first.endPoint.AlmostEquals(second.startPoint, eps);
    }

    namespace
    {
        // 溢出安全的二维方向归一化：先以最大绝对分量缩放，再求长度与单位向量，
        // 避免极大有限坐标在长度平方或叉积中溢出。零长度向量失败。
        [[nodiscard]] bool OverflowSafeUnit(const SCVector2d& vector, SCVector2d& unit)
        {
            const double ax = std::abs(vector.x);
            const double ay = std::abs(vector.y);
            const double maxAbs = std::max(ax, ay);
            if (!std::isfinite(maxAbs) || maxAbs <= 0.0)
            {
                return false;
            }
            const double sx = vector.x / maxAbs;
            const double sy = vector.y / maxAbs;
            const double length = std::sqrt(sx * sx + sy * sy);
            if (!std::isfinite(length) || length <= 0.0)
            {
                return false;
            }
            unit = SCVector2d{sx / length, sy / length};
            return unit.IsValid();
        }

        // 以 first.StartPoint() 为原点、unitFirst 为轴的溢出安全局部投影。
        // unitFirst 分量在 [-1,1]，故点积结果不超过原始坐标量级，不会溢出。
        [[nodiscard]] bool ProjectAlongAxis(const SCPoint2d& point, const SCPoint2d& origin, const SCVector2d& axis,
                                            double& projection)
        {
            const SCVector2d delta = point - origin;
            const double value = delta.x * axis.x + delta.y * axis.y;
            if (!std::isfinite(value))
            {
                return false;
            }
            projection = value;
            return true;
        }
    }  // namespace

    bool SCParallelSegmentProjectionTolerance2d::IsValid() const
    {
        // angularEpsilon 是单位方向叉积绝对值的无量纲阈值，合法范围严格为 (0, 1)。
        // angularEpsilon >= 1 会使任意方向被误判为平行，必须视为无效容差。
        return std::isfinite(angularEpsilon) && angularEpsilon > 0.0 && angularEpsilon < 1.0 &&
               std::isfinite(projectionEpsilon) && projectionEpsilon > 0.0;
    }

    SCParallelSegmentProjectionRelation2d ClassifyParallelSegmentProjection(
        const SCLineSegment2d& first,
        const SCLineSegment2d& second,
        const SCParallelSegmentProjectionTolerance2d& tolerance)
    {
        if (!tolerance.IsValid())
        {
            return SCParallelSegmentProjectionRelation2d::InvalidInput;
        }
        // 该 API 的有效性契约不同于 SCLineSegment2d::IsValid()：非零但短于
        // 默认几何 epsilon 的线段仍必须参与投影分类。
        if (!first.startPoint.IsValid() || !first.endPoint.IsValid() || !second.startPoint.IsValid() ||
            !second.endPoint.IsValid())
        {
            return SCParallelSegmentProjectionRelation2d::InvalidInput;
        }

        const SCVector2d firstDirection = first.endPoint - first.startPoint;
        const SCVector2d secondDirection = second.endPoint - second.startPoint;
        if (!firstDirection.IsValid() || !secondDirection.IsValid() ||
            (firstDirection.x == 0.0 && firstDirection.y == 0.0) ||
            (secondDirection.x == 0.0 && secondDirection.y == 0.0))
        {
            return SCParallelSegmentProjectionRelation2d::InvalidInput;
        }

        SCVector2d unitFirst{};
        SCVector2d unitSecond{};
        if (!OverflowSafeUnit(firstDirection, unitFirst) || !OverflowSafeUnit(secondDirection, unitSecond))
        {
            // 零长度线段或非有限中间计算均视为无效输入。
            return SCParallelSegmentProjectionRelation2d::InvalidInput;
        }

        // 不得计算 eps * Length(u) * Length(v)：单位向量叉积绝对值不超过 1。
        const double directionCross = std::abs(Cross(unitFirst, unitSecond));
        if (!std::isfinite(directionCross))
        {
            return SCParallelSegmentProjectionRelation2d::InvalidInput;
        }
        if (directionCross > tolerance.angularEpsilon)
        {
            return SCParallelSegmentProjectionRelation2d::NonParallel;
        }

        // 投影区间：first 为 [0, L1]（unitFirst 即 first 方向），second 为 [min2, max2]。
        double firstExtent = 0.0;
        if (!ProjectAlongAxis(first.endPoint, first.startPoint, unitFirst, firstExtent))
        {
            return SCParallelSegmentProjectionRelation2d::InvalidInput;
        }
        double secondStart = 0.0;
        double secondEnd = 0.0;
        if (!ProjectAlongAxis(second.startPoint, first.startPoint, unitFirst, secondStart) ||
            !ProjectAlongAxis(second.endPoint, first.startPoint, unitFirst, secondEnd))
        {
            return SCParallelSegmentProjectionRelation2d::InvalidInput;
        }

        const double firstMin = 0.0;
        const double firstMax = std::max(0.0, firstExtent);
        const double secondMin = std::min(secondStart, secondEnd);
        const double secondMax = std::max(secondStart, secondEnd);
        const double overlap = std::min(firstMax, secondMax) - std::max(firstMin, secondMin);
        if (!std::isfinite(overlap))
        {
            return SCParallelSegmentProjectionRelation2d::InvalidInput;
        }

        if (overlap < -tolerance.projectionEpsilon)
        {
            return SCParallelSegmentProjectionRelation2d::NoPositiveLengthIntersection;
        }
        if (overlap <= tolerance.projectionEpsilon)
        {
            return SCParallelSegmentProjectionRelation2d::EndpointTouch;
        }
        return SCParallelSegmentProjectionRelation2d::PositiveLengthIntersection;
    }
}  // namespace Geometry
