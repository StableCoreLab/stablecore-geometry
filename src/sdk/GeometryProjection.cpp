#include "sdk/GeometryProjection.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "common/Epsilon.h"
#include "sdk/GeometryMetrics.h"

namespace geometry::sdk
{
namespace
{
[[nodiscard]] double Clamp01(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

[[nodiscard]] double NormalizeAngle(double angle)
{
    constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;
    angle = std::fmod(angle, kTwoPi);
    if (angle < 0.0)
    {
        angle += kTwoPi;
    }
    return angle;
}

[[nodiscard]] double SignedAngleDelta(double startAngle, double endAngle)
{
    return NormalizeAngle(endAngle - startAngle);
}

[[nodiscard]] double ParameterOnArc(const ArcSegment2d& segment, double angle)
{
    if (segment.sweepAngle >= 0.0)
    {
        return SignedAngleDelta(segment.startAngle, angle) / segment.sweepAngle;
    }

    return -SignedAngleDelta(angle, segment.startAngle) / segment.sweepAngle;
}

[[nodiscard]] Vector2d LeftNormal(const Vector2d& tangent)
{
    return Vector2d{-tangent.y, tangent.x};
}
} // namespace

SegmentProjection2d ProjectPointToLineSegment(
    const Point2d& point,
    const LineSegment2d& segment,
    bool clampToSegment)
{
    if (!segment.IsValid())
    {
        return SegmentProjection2d{
            segment.startPoint,
            0.0,
            DistanceSquared(point, segment.startPoint),
            true};
    }

    const Vector2d direction = segment.endPoint - segment.startPoint;
    const double lengthSquared = direction.LengthSquared();
    const double rawParameter = geometry::Dot(point - segment.startPoint, direction) / lengthSquared;
    const double parameter = clampToSegment ? Clamp01(rawParameter) : rawParameter;
    const Point2d projectedPoint = segment.PointAt(parameter);

    return SegmentProjection2d{
        projectedPoint,
        parameter,
            DistanceSquared(point, projectedPoint),
        rawParameter >= -geometry::kDefaultEpsilon && rawParameter <= 1.0 + geometry::kDefaultEpsilon};
}

SegmentProjection2d ProjectPointToArcSegment(
    const Point2d& point,
    const ArcSegment2d& segment,
    bool clampToSegment)
{
    if (!segment.IsValid())
    {
        return SegmentProjection2d{};
    }

    const Vector2d radial = point - segment.center;
    double angle = std::atan2(radial.y, radial.x);
    const double rawParameter = ParameterOnArc(segment, angle);
    const double parameter = clampToSegment ? Clamp01(rawParameter) : rawParameter;
    const Point2d projectedPoint = segment.PointAt(parameter);

    return SegmentProjection2d{
        projectedPoint,
        parameter,
        DistanceSquared(point, projectedPoint),
        rawParameter >= -geometry::kDefaultEpsilon && rawParameter <= 1.0 + geometry::kDefaultEpsilon};
}

SegmentProjection2d ProjectPointToSegment(
    const Point2d& point,
    const Segment2d& segment,
    bool clampToSegment)
{
    if (segment.Kind() == SegmentKind2::Line)
    {
        return ProjectPointToLineSegment(point, static_cast<const LineSegment2d&>(segment), clampToSegment);
    }

    return ProjectPointToArcSegment(point, static_cast<const ArcSegment2d&>(segment), clampToSegment);
}

SegmentProjection2d ProjectPointToSegment(
    const Point2d& point,
    const Point2d& segmentStart,
    const Point2d& segmentEnd,
    bool clampToSegment)
{
    return ProjectPointToLineSegment(point, LineSegment2d(segmentStart, segmentEnd), clampToSegment);
}

double ParameterAtLength(const LineSegment2d& segment, double length, bool clampToSegment)
{
    const double totalLength = segment.Length();
    if (totalLength <= geometry::kDefaultEpsilon)
    {
        return 0.0;
    }

    if (clampToSegment)
    {
        length = std::max(0.0, std::min(length, totalLength));
    }
    return length / totalLength;
}

double ParameterAtLength(const ArcSegment2d& segment, double length, bool clampToSegment)
{
    const double totalLength = segment.Length();
    if (totalLength <= geometry::kDefaultEpsilon)
    {
        return 0.0;
    }

    if (clampToSegment)
    {
        length = std::max(0.0, std::min(length, totalLength));
    }
    return length / totalLength;
}

Vector2d TangentAt(const LineSegment2d& segment, double)
{
    const Vector2d direction = segment.endPoint - segment.startPoint;
    const double length = direction.Length();
    if (length <= geometry::kDefaultEpsilon)
    {
        return Vector2d{};
    }
    return direction / length;
}

Vector2d TangentAt(const ArcSegment2d& segment, double parameter)
{
    const Point2d point = segment.PointAt(parameter);
    Vector2d radial = point - segment.center;
    const double radialLength = radial.Length();
    if (radialLength <= geometry::kDefaultEpsilon)
    {
        return Vector2d{};
    }
    radial = radial / radialLength;
    return segment.Direction() == ArcDirection::CounterClockwise ? Vector2d{-radial.y, radial.x}
                                                                 : Vector2d{radial.y, -radial.x};
}

Vector2d TangentAt(const Segment2d& segment, double parameter)
{
    if (segment.Kind() == SegmentKind2::Line)
    {
        return TangentAt(static_cast<const LineSegment2d&>(segment), parameter);
    }

    return TangentAt(static_cast<const ArcSegment2d&>(segment), parameter);
}

Vector2d NormalAt(const LineSegment2d& segment, double parameter)
{
    return LeftNormal(TangentAt(segment, parameter));
}

Vector2d NormalAt(const ArcSegment2d& segment, double parameter)
{
    return LeftNormal(TangentAt(segment, parameter));
}

Vector2d NormalAt(const Segment2d& segment, double parameter)
{
    return LeftNormal(TangentAt(segment, parameter));
}
} // namespace geometry::sdk
