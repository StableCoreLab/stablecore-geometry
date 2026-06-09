#pragma once

#include <cmath>
#include <memory>

#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCPolyline2d.h"

namespace Geometry::Detail
{
    struct RingMoment2d
    {
        double signedArea{0.0};
        double firstMomentX{0.0};
        double firstMomentY{0.0};
    };

    inline void AccumulateLineSegmentMoment(const SCPoint2d& startPoint, const SCPoint2d& endPoint, RingMoment2d& total)
    {
        const double cross = startPoint.x * endPoint.y - endPoint.x * startPoint.y;

        total.signedArea += cross / 2.0;
        total.firstMomentX += (endPoint.y - startPoint.y) *
                              (startPoint.x * startPoint.x + startPoint.x * endPoint.x + endPoint.x * endPoint.x) / 6.0;
        total.firstMomentY += (startPoint.x - endPoint.x) *
                              (startPoint.y * startPoint.y + startPoint.y * endPoint.y + endPoint.y * endPoint.y) / 6.0;
    }

    inline double ArcFirstMomentXPrimitive(double centerX, double radius, double angle)
    {
        const double sinAngle = std::sin(angle);
        const double cosAngle = std::cos(angle);
        return (centerX * centerX * radius * sinAngle + centerX * radius * radius * (angle + sinAngle * cosAngle) +
                radius * radius * radius * (sinAngle - sinAngle * sinAngle * sinAngle / 3.0)) /
               2.0;
    }

    inline double ArcFirstMomentYPrimitive(double centerY, double radius, double angle)
    {
        const double sinAngle = std::sin(angle);
        const double cosAngle = std::cos(angle);
        return (-centerY * centerY * radius * cosAngle + centerY * radius * radius * (angle - sinAngle * cosAngle) +
                radius * radius * radius * (-cosAngle + cosAngle * cosAngle * cosAngle / 3.0)) /
               2.0;
    }

    inline void AccumulateArcSegmentMoment(const SCArcSegment2d& arc, RingMoment2d& total)
    {
        const double cx = arc.center.x;
        const double cy = arc.center.y;
        const double radius = arc.radius;
        const double startAngle = arc.startAngle;
        const double sweep = arc.sweepAngle;
        const double endAngle = startAngle + sweep;

        const double sinStart = std::sin(startAngle);
        const double cosStart = std::cos(startAngle);
        const double sinEnd = std::sin(endAngle);
        const double cosEnd = std::cos(endAngle);

        total.signedArea +=
            (cx * radius * (sinEnd - sinStart) - cy * radius * (cosEnd - cosStart) + radius * radius * sweep) / 2.0;

        total.firstMomentX +=
            ArcFirstMomentXPrimitive(cx, radius, endAngle) - ArcFirstMomentXPrimitive(cx, radius, startAngle);
        total.firstMomentY +=
            ArcFirstMomentYPrimitive(cy, radius, endAngle) - ArcFirstMomentYPrimitive(cy, radius, startAngle);
    }

    inline RingMoment2d ComputeRingMoment(const SCPolyline2d& ring)
    {
        RingMoment2d total{};
        for (std::size_t i = 0; i < ring.SegmentCount(); ++i)
        {
            std::unique_ptr<ISCSegment2d> segment = ring.SegmentAt(i);
            if (segment == nullptr)
            {
                continue;
            }

            switch (segment->Kind())
            {
                case SCSegmentKind2::Line:
                    AccumulateLineSegmentMoment(segment->StartPoint(), segment->EndPoint(), total);
                    break;
                case SCSegmentKind2::Arc:
                    AccumulateArcSegmentMoment(static_cast<const SCArcSegment2d&>(*segment), total);
                    break;
                default:
                    break;
            }
        }

        return total;
    }

    inline double ComputeSignedArea(const SCPolyline2d& ring)
    {
        if (!ring.IsClosed() || !ring.IsValid() || ring.SegmentCount() == 0)
        {
            return 0.0;
        }

        return ComputeRingMoment(ring).signedArea;
    }
}  // namespace Geometry::Detail
