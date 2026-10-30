#include "Core/Editing.h"

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "Core/Projection.h"
#include "Core/Relation.h"
#include "Core/ShapeOps.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] bool IsValidEditingTolerance(double eps)
        {
            return eps > 0.0 && std::isfinite(eps);
        }

        [[nodiscard]] std::vector<SCPoint2d> NormalizedPoints(const SCPolyline2d& polyline, double eps)
        {
            std::vector<SCPoint2d> points;
            if (polyline.PointCount() == 0)
            {
                return points;
            }

            points.reserve(polyline.PointCount());
            for (std::size_t i = 0; i < polyline.PointCount(); ++i)
            {
                const SCPoint2d point = polyline.VertexAt(i);
                if (!points.empty() && points.back().AlmostEquals(point, eps))
                {
                    continue;
                }
                points.push_back(point);
            }

            if (polyline.IsClosed() && points.size() > 1 && points.front().AlmostEquals(points.back(), eps))
            {
                points.pop_back();
            }
            return points;
        }
    }  // namespace

    SCPolyline2d Normalize(const SCPolyline2d& polyline, double eps)
    {
        if (!IsValidEditingTolerance(eps))
        {
            return {};
        }
        if (!polyline.HasNonLineSegments())
        {
            return SCPolyline2d(NormalizedPoints(polyline, eps),
                                polyline.IsClosed() ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
        }
        std::vector<std::shared_ptr<ISCSegment2d>> segments;
        segments.reserve(polyline.SegmentCount());
        for (std::size_t i = 0; i < polyline.SegmentCount(); ++i)
        {
            std::unique_ptr<ISCSegment2d> segment = polyline.SegmentAt(i);
            if (segment == nullptr)
            {
                return {};
            }
            if (segment->Kind() == SCSegmentKind2::Line &&
                segment->StartPoint().AlmostEquals(segment->EndPoint(), eps))
            {
                continue;
            }
            segments.push_back(std::shared_ptr<ISCSegment2d>(std::move(segment)));
        }
        return SCPolyline2d(std::move(segments), polyline.IsClosed()
            ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
    }

    SCPolygon2d Normalize(const SCPolygon2d& polygon, double eps)
    {
        if (!IsValidEditingTolerance(eps))
        {
            return {};
        }
        std::vector<SCPolyline2d> holes;
        holes.reserve(polygon.HoleCount());
        for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
        {
            holes.push_back(Normalize(polygon.HoleAt(i), eps));
        }
        return SCPolygon2d(Normalize(polygon.OuterRing(), eps), std::move(holes));
    }

    SCPolyline2d InsertPoint(const SCPolyline2d& polyline, const SCPoint2d& point, double eps)
    {
        if (!IsValidEditingTolerance(eps))
        {
            return {};
        }
        if (polyline.HasNonLineSegments())
        {
            std::vector<std::shared_ptr<ISCSegment2d>> segments;
            segments.reserve(polyline.SegmentCount() + 1);
            bool inserted = false;
            for (std::size_t i = 0; i < polyline.SegmentCount(); ++i)
            {
                std::unique_ptr<ISCSegment2d> segment = polyline.SegmentAt(i);
                bool splitThisSegment = false;
                if (segment == nullptr)
                {
                    return {};
                }
                if (!inserted && !point.AlmostEquals(segment->StartPoint(), eps) &&
                    !point.AlmostEquals(segment->EndPoint(), eps))
                {
                    if (segment->Kind() == SCSegmentKind2::Line &&
                        LocatePoint(point, *segment, eps) == SCPointContainment2d::OnBoundary)
                    {
                        segments.push_back(std::make_shared<SCLineSegment2d>(segment->StartPoint(), point));
                        segments.push_back(std::make_shared<SCLineSegment2d>(point, segment->EndPoint()));
                        inserted = true;
                        splitThisSegment = true;
                    } else if (segment->Kind() == SCSegmentKind2::Arc)
                    {
                        const auto& arc = static_cast<const SCArcSegment2d&>(*segment);
                        const SCSegmentProjection2d projection =
                            ProjectPointToArcSegment(point, arc, false, eps);
                        if (projection.IsValid() && projection.isOnSegment &&
                            projection.parameter > 0.0 && projection.parameter < 1.0 &&
                            std::hypot(point.x - projection.point.x,
                                       point.y - projection.point.y) <= eps)
                        {
                            const double firstSweep = arc.sweepAngle * projection.parameter;
                            segments.push_back(std::make_shared<SCArcSegment2d>(
                                arc.center, arc.radius, arc.startAngle, firstSweep));
                            segments.push_back(std::make_shared<SCArcSegment2d>(
                                arc.center, arc.radius, arc.startAngle + firstSweep,
                                arc.sweepAngle - firstSweep));
                            inserted = true;
                            splitThisSegment = true;
                        }
                    }
                }
                if (!splitThisSegment)
                {
                    segments.push_back(std::shared_ptr<ISCSegment2d>(std::move(segment)));
                }
            }
            return SCPolyline2d(std::move(segments), polyline.IsClosed()
                ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
        }

        std::vector<SCPoint2d> points;
        points.reserve(polyline.PointCount() + 1);

        bool inserted = false;
        const std::size_t n = polyline.PointCount();
        const std::size_t segmentCount = polyline.IsClosed() ? n : (n > 0 ? n - 1 : 0);
        for (std::size_t i = 0; i < n; ++i)
        {
            points.push_back(polyline.VertexAt(i));
            if (!inserted && i < segmentCount)
            {
                const SCPoint2d next = polyline.VertexAt((i + 1) % n);
                if (LocatePoint(point, SCLineSegment2d(polyline.VertexAt(i), next), eps) ==
                        SCPointContainment2d::OnBoundary &&
                    !point.AlmostEquals(polyline.VertexAt(i), eps) && !point.AlmostEquals(next, eps))
                {
                    points.push_back(point);
                    inserted = true;
                }
            }
        }

        return SCPolyline2d(std::move(points), polyline.IsClosed() ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
    }

    SCPolygon2d InsertPoint(const SCPolygon2d& polygon, const SCPoint2d& point, double eps)
    {
        if (!IsValidEditingTolerance(eps))
        {
            return {};
        }
        std::vector<SCPolyline2d> holes;
        holes.reserve(polygon.HoleCount());
        for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
        {
            holes.push_back(InsertPoint(polygon.HoleAt(i), point, eps));
        }
        return SCPolygon2d(InsertPoint(polygon.OuterRing(), point, eps), std::move(holes));
    }
}  // namespace Geometry

