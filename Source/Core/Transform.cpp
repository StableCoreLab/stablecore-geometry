#include "Core/Transform.h"

#include <cmath>
#include <memory>
#include <vector>

#include "Core/Metrics.h"
#include "Core/ShapeOps.h"
#include "Support/Epsilon.h"
#include "Support/Geometry2d/Predicate2.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] SCPoint2d RotatePoint(const SCPoint2d& point, const SCPoint2d& origin, double angle)
        {
            const double c = std::cos(angle);
            const double s = std::sin(angle);
            const SCVector2d delta = point - origin;
            return SCPoint2d{origin.x + delta.x * c - delta.y * s, origin.y + delta.x * s + delta.y * c};
        }

        [[nodiscard]] SCPoint2d MirrorPoint(const SCPoint2d& point, const SCPoint2d& linePoint, const SCVector2d& lineDir)
        {
            const double lengthSquared = lineDir.LengthSquared();
            if (lengthSquared <= Geometry::kTransformDefaultEpsilon)
            {
                return point;
            }
            const SCVector2d local = point - linePoint;
            const SCVector2d unit = lineDir / std::sqrt(lengthSquared);
            const double along = Dot(local, unit);
            const SCPoint2d projected = linePoint + unit * along;
            return projected + (projected - point);
        }

        template <typename LineTransform, typename ArcTransform>
        [[nodiscard]] SCPolyline2d TransformPolylineSegments(const SCPolyline2d& polyline,
                                                           LineTransform&& lineTransform,
                                                           ArcTransform&& arcTransform)
        {
            std::vector<std::shared_ptr<ISCSegment2d>> segments;
            segments.reserve(polyline.SegmentCount());
            for (std::size_t i = 0; i < polyline.SegmentCount(); ++i)
            {
                std::unique_ptr<ISCSegment2d> segment = polyline.SegmentAt(i);
                if (segment == nullptr)
                {
                    segments.push_back(nullptr);
                    continue;
                }

                switch (segment->Kind())
                {
                    case SCSegmentKind2::Line:
                        segments.push_back(std::make_shared<SCLineSegment2d>(
                            lineTransform(static_cast<const SCLineSegment2d&>(*segment))));
                        break;
                    case SCSegmentKind2::Arc:
                        segments.push_back(
                            std::make_shared<SCArcSegment2d>(arcTransform(static_cast<const SCArcSegment2d&>(*segment))));
                        break;
                    default:
                        segments.push_back(std::shared_ptr<ISCSegment2d>(segment->Clone().release()));
                        break;
                }
            }

            return SCPolyline2d(std::move(segments),
                              polyline.IsClosed() ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
        }
    }  // namespace

    SCPoint2d Translate(const SCPoint2d& point, const SCVector2d& offset)
    {
        return point + offset;
    }

    SCLineSegment2d Translate(const SCLineSegment2d& segment, const SCVector2d& offset)
    {
        return SCLineSegment2d(segment.startPoint + offset, segment.endPoint + offset);
    }

    SCArcSegment2d Translate(const SCArcSegment2d& segment, const SCVector2d& offset)
    {
        return SCArcSegment2d(segment.center + offset, segment.radius, segment.startAngle, segment.sweepAngle);
    }

    SCPolyline2d Translate(const SCPolyline2d& polyline, const SCVector2d& offset)
    {
        return TransformPolylineSegments(
            polyline,
            [&](const SCLineSegment2d& segment) { return Translate(segment, offset); },
            [&](const SCArcSegment2d& segment) { return Translate(segment, offset); });
    }

    SCPolygon2d Translate(const SCPolygon2d& polygon, const SCVector2d& offset)
    {
        std::vector<SCPolyline2d> holes;
        holes.reserve(polygon.HoleCount());
        for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
        {
            holes.push_back(Translate(polygon.HoleAt(i), offset));
        }
        return SCPolygon2d(Translate(polygon.OuterRing(), offset), std::move(holes));
    }

    SCMultiPolyline2d Translate(const SCMultiPolyline2d& polylines, const SCVector2d& offset)
    {
        std::vector<SCPolyline2d> result;
        result.reserve(polylines.Count());
        for (std::size_t i = 0; i < polylines.Count(); ++i)
        {
            result.push_back(Translate(polylines[i], offset));
        }
        return SCMultiPolyline2d(std::move(result));
    }

    SCMultiPolygon2d Translate(const SCMultiPolygon2d& polygons, const SCVector2d& offset)
    {
        std::vector<SCPolygon2d> result;
        result.reserve(polygons.Count());
        for (std::size_t i = 0; i < polygons.Count(); ++i)
        {
            result.push_back(Translate(polygons[i], offset));
        }
        return SCMultiPolygon2d(std::move(result));
    }

    SCPoint2d Rotate(const SCPoint2d& point, const SCPoint2d& origin, double angleRadians)
    {
        return RotatePoint(point, origin, angleRadians);
    }

    SCLineSegment2d Rotate(const SCLineSegment2d& segment, const SCPoint2d& origin, double angleRadians)
    {
        return SCLineSegment2d(RotatePoint(segment.startPoint, origin, angleRadians),
                             RotatePoint(segment.endPoint, origin, angleRadians));
    }

    SCArcSegment2d Rotate(const SCArcSegment2d& segment, const SCPoint2d& origin, double angleRadians)
    {
        return SCArcSegment2d(RotatePoint(segment.center, origin, angleRadians),
                            segment.radius,
                            segment.startAngle + angleRadians,
                            segment.sweepAngle);
    }

    SCPolyline2d Rotate(const SCPolyline2d& polyline, const SCPoint2d& origin, double angleRadians)
    {
        return TransformPolylineSegments(
            polyline,
            [&](const SCLineSegment2d& segment) { return Rotate(segment, origin, angleRadians); },
            [&](const SCArcSegment2d& segment) { return Rotate(segment, origin, angleRadians); });
    }

    SCPolygon2d Rotate(const SCPolygon2d& polygon, const SCPoint2d& origin, double angleRadians)
    {
        std::vector<SCPolyline2d> holes;
        holes.reserve(polygon.HoleCount());
        for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
        {
            holes.push_back(Rotate(polygon.HoleAt(i), origin, angleRadians));
        }
        return SCPolygon2d(Rotate(polygon.OuterRing(), origin, angleRadians), std::move(holes));
    }

    SCPoint2d Mirror(const SCPoint2d& point, const SCPoint2d& linePoint, const SCVector2d& lineDir)
    {
        return MirrorPoint(point, linePoint, lineDir);
    }

    SCLineSegment2d Mirror(const SCLineSegment2d& segment, const SCPoint2d& linePoint, const SCVector2d& lineDir)
    {
        return SCLineSegment2d(MirrorPoint(segment.startPoint, linePoint, lineDir),
                             MirrorPoint(segment.endPoint, linePoint, lineDir));
    }

    SCArcSegment2d Mirror(const SCArcSegment2d& segment, const SCPoint2d& linePoint, const SCVector2d& lineDir)
    {
        const SCPoint2d newCenter = MirrorPoint(segment.center, linePoint, lineDir);
        return SCArcSegment2d(newCenter,
                            segment.radius,
                            2.0 * std::atan2(lineDir.y, lineDir.x) - segment.startAngle,
                            -segment.sweepAngle);
    }

    SCPolyline2d Mirror(const SCPolyline2d& polyline, const SCPoint2d& linePoint, const SCVector2d& lineDir)
    {
        return TransformPolylineSegments(
            polyline,
            [&](const SCLineSegment2d& segment) { return Mirror(segment, linePoint, lineDir); },
            [&](const SCArcSegment2d& segment) { return Mirror(segment, linePoint, lineDir); });
    }

    SCPolygon2d Mirror(const SCPolygon2d& polygon, const SCPoint2d& linePoint, const SCVector2d& lineDir)
    {
        std::vector<SCPolyline2d> holes;
        holes.reserve(polygon.HoleCount());
        for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
        {
            holes.push_back(Reverse(Mirror(polygon.HoleAt(i), linePoint, lineDir)));
        }
        return SCPolygon2d(Reverse(Mirror(polygon.OuterRing(), linePoint, lineDir)), std::move(holes));
    }

    SCPoint2d Stretch(const SCPoint2d& point, const SCBox2d& region, const SCVector2d& offset)
    {
        return Contains(region, point) ? point + offset : point;
    }

    SCLineSegment2d Stretch(const SCLineSegment2d& segment, const SCBox2d& region, const SCVector2d& offset)
    {
        return SCLineSegment2d(Stretch(segment.startPoint, region, offset), Stretch(segment.endPoint, region, offset));
    }

    SCArcSegment2d Stretch(const SCArcSegment2d& segment, const SCBox2d& region, const SCVector2d& offset)
    {
        const SCPoint2d start = Stretch(segment.StartPoint(), region, offset);
        const SCPoint2d end = Stretch(segment.EndPoint(), region, offset);
        if (start.AlmostEquals(segment.StartPoint()) && end.AlmostEquals(segment.EndPoint()))
        {
            return segment;
        }
        return SCArcSegment2d(segment.center,
                            segment.radius,
                            std::atan2(start.y - segment.center.y, start.x - segment.center.x),
                            segment.sweepAngle);
    }

    SCPolyline2d Stretch(const SCPolyline2d& polyline, const SCBox2d& region, const SCVector2d& offset)
    {
        return TransformPolylineSegments(
            polyline,
            [&](const SCLineSegment2d& segment) { return Stretch(segment, region, offset); },
            [&](const SCArcSegment2d& segment) { return Stretch(segment, region, offset); });
    }

    SCPolygon2d Stretch(const SCPolygon2d& polygon, const SCBox2d& region, const SCVector2d& offset)
    {
        std::vector<SCPolyline2d> holes;
        holes.reserve(polygon.HoleCount());
        for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
        {
            holes.push_back(Stretch(polygon.HoleAt(i), region, offset));
        }
        return SCPolygon2d(Stretch(polygon.OuterRing(), region, offset), std::move(holes));
    }
}  // namespace Geometry
