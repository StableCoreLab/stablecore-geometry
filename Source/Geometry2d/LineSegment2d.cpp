#include "Geometry2d/SCLineSegment2d.h"

#include <algorithm>
#include <memory>
#include <cmath>
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
        return IsValid(Geometry::kDefaultEpsilon);
    }

    bool SCLineSegment2d::IsValid(double eps) const
    {
        if (!(eps > 0.0) || !std::isfinite(eps) || !startPoint.IsValid() || !endPoint.IsValid())
        {
            return false;
        }
        const double length = Length();
        return std::isfinite(length) && length > eps;
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
        return Bounds(Geometry::kDefaultEpsilon);
    }

    SCBox2d SCLineSegment2d::Bounds(double eps) const
    {
        if (!IsValid(eps))
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

    SCPoint2d SCLineSegment2d::PointAt(double parameter, double eps) const
    {
        if (!std::isfinite(eps) || eps <= 0.0 || !std::isfinite(parameter) || !IsValid(eps) ||
            !std::isfinite(Length()))
        {
            return startPoint;
        }
        const SCPoint2d point{std::lerp(startPoint.x, endPoint.x, parameter),
                              std::lerp(startPoint.y, endPoint.y, parameter)};
        return point.IsValid() ? point : startPoint;
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

    SCPoint2d SCLineSegment2d::PointAtLength(double distanceFromStart, bool clampToSegment, double eps) const
    {
        if (!std::isfinite(distanceFromStart) || !std::isfinite(eps) || eps <= 0.0 || !IsValid(eps))
        {
            return startPoint;
        }
        const double length = Length();
        if (!std::isfinite(length) || length <= eps)
        {
            return startPoint;
        }
        if (clampToSegment)
        {
            distanceFromStart = std::clamp(distanceFromStart, 0.0, length);
        }
        return PointAt(distanceFromStart / length, eps);
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
