#include "sdk/LineSegment2d.h"

#include <memory>
#include <sstream>

namespace geometry::sdk
{
LineSegment2d::LineSegment2d(const Point2d& startPointValue, const Point2d& endPointValue)
    : startPoint(startPointValue), endPoint(endPointValue)
{
}

LineSegment2d LineSegment2d::FromEndpoints(
    const Point2d& startPointValue,
    const Point2d& endPointValue)
{
    return LineSegment2d(startPointValue, endPointValue);
}

SegmentKind2 LineSegment2d::Kind() const
{
    return SegmentKind2::Line;
}

bool LineSegment2d::IsValid() const
{
    return startPoint.IsValid() && endPoint.IsValid() && !startPoint.AlmostEquals(endPoint);
}

Point2d LineSegment2d::StartPoint() const
{
    return startPoint;
}

Point2d LineSegment2d::EndPoint() const
{
    return endPoint;
}

double LineSegment2d::Length() const
{
    return (endPoint - startPoint).Length();
}

Box2d LineSegment2d::Bounds() const
{
    if (!IsValid())
    {
        return Box2d{};
    }

    Box2d box;
    box.ExpandToInclude(startPoint);
    box.ExpandToInclude(endPoint);
    return box;
}

Point2d LineSegment2d::PointAt(double parameter) const
{
    return PointAtLength(parameter * Length(), false);
}

Point2d LineSegment2d::PointAtLength(double distanceFromStart, bool clampToSegment) const
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
        }
        else if (distanceFromStart > length)
        {
            distanceFromStart = length;
        }
    }

    const double ratio = distanceFromStart / length;
    return Point2d{
        startPoint.x + (endPoint.x - startPoint.x) * ratio,
        startPoint.y + (endPoint.y - startPoint.y) * ratio};
}

bool LineSegment2d::AlmostEquals(const LineSegment2d& other, double eps) const
{
    return startPoint.AlmostEquals(other.startPoint, eps) &&
           endPoint.AlmostEquals(other.endPoint, eps);
}

std::string LineSegment2d::DebugString() const
{
    std::ostringstream stream;
    stream << "LineSegment2d{start=" << startPoint.DebugString()
           << ", end=" << endPoint.DebugString() << "}";
    return stream.str();
}

std::unique_ptr<Segment2d> LineSegment2d::Clone() const
{
    return std::make_unique<LineSegment2d>(*this);
}
} // namespace geometry::sdk
