#include "Core/Editing.h"

#include <vector>

#include "Core/Relation.h"
#include "Core/ShapeOps.h"

namespace Geometry
{
    namespace
    {
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
                const SCPoint2d point = polyline.PointAt(i);
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
        return SCPolyline2d(NormalizedPoints(polyline, eps),
                          polyline.IsClosed() ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
    }

    SCPolygon2d Normalize(const SCPolygon2d& polygon, double eps)
    {
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
        std::vector<SCPoint2d> points;
        points.reserve(polyline.PointCount() + 1);

        bool inserted = false;
        const std::size_t n = polyline.PointCount();
        const std::size_t segmentCount = polyline.IsClosed() ? n : (n > 0 ? n - 1 : 0);
        for (std::size_t i = 0; i < n; ++i)
        {
            points.push_back(polyline.PointAt(i));
            if (!inserted && i < segmentCount)
            {
                const SCPoint2d next = polyline.PointAt((i + 1) % n);
                if (LocatePoint(point, SCLineSegment2d(polyline.PointAt(i), next), eps) ==
                        SCPointContainment2d::OnBoundary &&
                    !point.AlmostEquals(polyline.PointAt(i), eps) && !point.AlmostEquals(next, eps))
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
        return SCPolygon2d(InsertPoint(polygon.OuterRing(), point, eps));
    }
}  // namespace Geometry

