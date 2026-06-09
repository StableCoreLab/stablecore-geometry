#include "Core/Projection.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "Core/Metrics.h"
#include "Support/Geometry2d/Predicate2.h"
#include "Support/Epsilon.h"

namespace Geometry
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

        [[nodiscard]] double ParameterOnArc(const SCArcSegment2d& segment, double angle)
        {
            if (segment.sweepAngle >= 0.0)
            {
                return SignedAngleDelta(segment.startAngle, angle) / segment.sweepAngle;
            }

            return -SignedAngleDelta(angle, segment.startAngle) / segment.sweepAngle;
        }

        [[nodiscard]] SCVector2d LeftNormal(const SCVector2d& tangent)
        {
            return SCVector2d{-tangent.y, tangent.x};
        }
    }  // namespace

    SCSegmentProjection2d ProjectPointToLineSegment(const SCPoint2d& point,
                                                  const SCLineSegment2d& segment,
                                                  bool clampToSegment)
    {
        if (!segment.IsValid())
        {
            return SCSegmentProjection2d{segment.startPoint, 0.0, DistanceSquared(point, segment.startPoint), true};
        }

        const SCVector2d direction = segment.endPoint - segment.startPoint;
        const double lengthSquared = direction.LengthSquared();
        const double rawParameter = Geometry::Dot(point - segment.startPoint, direction) / lengthSquared;
        const double parameter = clampToSegment ? Clamp01(rawParameter) : rawParameter;
        const SCPoint2d projectedPoint = segment.PointAt(parameter);

        return SCSegmentProjection2d{projectedPoint,
                                   parameter,
                                   DistanceSquared(point, projectedPoint),
                                   rawParameter >= -Geometry::kProjectionDefaultEpsilon &&
                                       rawParameter <= 1.0 + Geometry::kProjectionDefaultEpsilon};
    }

    SCSegmentProjection2d ProjectPointToArcSegment(const SCPoint2d& point, const SCArcSegment2d& segment, bool clampToSegment)
    {
        if (!segment.IsValid())
        {
            return SCSegmentProjection2d{};
        }

        const SCVector2d radial = point - segment.center;
        double angle = std::atan2(radial.y, radial.x);
        const double rawParameter = ParameterOnArc(segment, angle);
        const double parameter = clampToSegment ? Clamp01(rawParameter) : rawParameter;
        const SCPoint2d projectedPoint = segment.PointAt(parameter);

        return SCSegmentProjection2d{projectedPoint,
                                   parameter,
                                   DistanceSquared(point, projectedPoint),
                                   rawParameter >= -Geometry::kProjectionDefaultEpsilon &&
                                       rawParameter <= 1.0 + Geometry::kProjectionDefaultEpsilon};
    }

    SCSegmentProjection2d ProjectPointToSegment(const SCPoint2d& point, const ISCSegment2d& segment, bool clampToSegment)
    {
        if (segment.Kind() == SCSegmentKind2::Line)
        {
            return ProjectPointToLineSegment(point, static_cast<const SCLineSegment2d&>(segment), clampToSegment);
        }

        return ProjectPointToArcSegment(point, static_cast<const SCArcSegment2d&>(segment), clampToSegment);
    }

    double ParameterAtLength(const SCLineSegment2d& segment, double length, bool clampToSegment)
    {
        const double totalLength = segment.Length();
        if (totalLength <= Geometry::kProjectionDefaultEpsilon)
        {
            return 0.0;
        }

        if (clampToSegment)
        {
            length = std::max(0.0, std::min(length, totalLength));
        }
        return length / totalLength;
    }

    double ParameterAtLength(const SCArcSegment2d& segment, double length, bool clampToSegment)
    {
        const double totalLength = segment.Length();
        if (totalLength <= Geometry::kProjectionDefaultEpsilon)
        {
            return 0.0;
        }

        if (clampToSegment)
        {
            length = std::max(0.0, std::min(length, totalLength));
        }
        return length / totalLength;
    }

    SCVector2d TangentAt(const SCLineSegment2d& segment, double)
    {
        const SCVector2d direction = segment.endPoint - segment.startPoint;
        const double length = direction.Length();
        if (length <= Geometry::kProjectionDefaultEpsilon)
        {
            return SCVector2d{};
        }
        return direction / length;
    }

    SCVector2d TangentAt(const SCArcSegment2d& segment, double parameter)
    {
        const SCPoint2d point = segment.PointAt(parameter);
        SCVector2d radial = point - segment.center;
        const double radialLength = radial.Length();
        if (radialLength <= Geometry::kProjectionDefaultEpsilon)
        {
            return SCVector2d{};
        }
        radial = radial / radialLength;
        return segment.Direction() == SCArcDirection::CounterClockwise ? SCVector2d{-radial.y, radial.x}
                                                                     : SCVector2d{radial.y, -radial.x};
    }

    SCVector2d TangentAt(const ISCSegment2d& segment, double parameter)
    {
        if (segment.Kind() == SCSegmentKind2::Line)
        {
            return TangentAt(static_cast<const SCLineSegment2d&>(segment), parameter);
        }

        return TangentAt(static_cast<const SCArcSegment2d&>(segment), parameter);
    }

    SCVector2d NormalAt(const SCLineSegment2d& segment, double parameter)
    {
        return LeftNormal(TangentAt(segment, parameter));
    }

    SCVector2d NormalAt(const SCArcSegment2d& segment, double parameter)
    {
        return LeftNormal(TangentAt(segment, parameter));
    }

    SCVector2d NormalAt(const ISCSegment2d& segment, double parameter)
    {
        return LeftNormal(TangentAt(segment, parameter));
    }
}  // namespace Geometry

