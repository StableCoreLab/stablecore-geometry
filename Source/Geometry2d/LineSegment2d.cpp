#include "Geometry2d/SCLineSegment2d.h"

#include <memory>
#include <sstream>

#include "Types/Geometry2d/SCVector2.h"

namespace Geometry
{
    SCLineSegment2d::SCLineSegment2d(const SCPoint2d& startPoint, const SCPoint2d& endPoint)
        : startPoint(startPoint), endPoint(endPoint)
    {
    }

    SCLineSegment2d SCLineSegment2d::FromEndpoints(const SCPoint2d& startPoint, const SCPoint2d& endPoint)
    {
        return SCLineSegment2d(startPoint, endPoint);
    }

    SCSegmentKind2 SCLineSegment2d::Kind() const
    {
        return SCSegmentKind2::Line;
    }

    bool SCLineSegment2d::IsValid() const
    {
        return startPoint.IsValid() && endPoint.IsValid() && !startPoint.AlmostEquals(endPoint);
    }

    SCPoint2d SCLineSegment2d::StartPoint() const
    {
        return startPoint;
    }

    SCPoint2d SCLineSegment2d::EndPoint() const
    {
        return endPoint;
    }

    double SCLineSegment2d::Length() const
    {
        return (endPoint - startPoint).Length();
    }

    SCBox2d SCLineSegment2d::Bounds() const
    {
        if (!IsValid())
        {
            return SCBox2d{};
        }

        SCBox2d box;
        box.ExpandToInclude(startPoint);
        box.ExpandToInclude(endPoint);
        return box;
    }

    SCPoint2d SCLineSegment2d::PointAt(double parameter) const
    {
        return PointAtLength(parameter * Length(), false);
    }

    SCPoint2d SCLineSegment2d::PointAtLength(double distanceFromStart, bool clampToSegment) const
    {
        if (!IsValid())
        {
            return startPoint;
        }

        const double length = Length();
        if (length <= 0.0)
        {
            return startPoint;
        }

        if (clampToSegment)
        {
            if (distanceFromStart < 0.0)
            {
                distanceFromStart = 0.0;
            } else if (distanceFromStart > length)
            {
                distanceFromStart = length;
            }
        }

        const double ratio = distanceFromStart / length;
        return SCPoint2d{startPoint.x + (endPoint.x - startPoint.x) * ratio,
                       startPoint.y + (endPoint.y - startPoint.y) * ratio};
    }

    bool SCLineSegment2d::AlmostEquals(const SCLineSegment2d& other, double eps) const
    {
        return startPoint.AlmostEquals(other.startPoint, eps) && endPoint.AlmostEquals(other.endPoint, eps);
    }

    std::string SCLineSegment2d::DebugString() const
    {
        std::ostringstream stream;
        stream << "SCLineSegment2d{start=" << startPoint.DebugString() << ", end=" << endPoint.DebugString() << "}";
        return stream.str();
    }

    std::unique_ptr<ISCSegment2d> SCLineSegment2d::Clone() const
    {
        return std::make_unique<SCLineSegment2d>(*this);
    }
}  // namespace Geometry
