#include "Core/Projection.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include "Core/Metrics.h"
#include "Brep/PolyhedronBody.h"
#include "Brep/PolyhedronFace3d.h"
#include "Brep/SCBrepBody.h"
#include "Brep/SCBrepEdge.h"
#include "Brep/SCBrepFace.h"
#include "Brep/SCBrepVertex.h"
#include "Geometry3d/ISCCurve3d.h"
#include "Geometry3d/ISCSurface.h"
#include "Geometry3d/TriangleMesh.h"
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

        [[nodiscard]] double ClampedArcProjectionParameter(const SCPoint2d& point,
                                                           const SCArcSegment2d& segment,
                                                           double rawParameter)
        {
            if (rawParameter > 1.0)
            {
                return Distance(point, segment.StartPoint()) <= Distance(point, segment.EndPoint()) ? 0.0 : 1.0;
            }
            return Clamp01(rawParameter);
        }

        [[nodiscard]] SCSegmentProjection2d InvalidSegmentProjection2d()
        {
            return {SCPoint2d{}, 0.0, std::numeric_limits<double>::quiet_NaN(), false};
        }

        [[nodiscard]] SCSegmentProjection3d InvalidSegmentProjection3d()
        {
            return {SCPoint3d{}, 0.0, std::numeric_limits<double>::quiet_NaN(), false};
        }

        [[nodiscard]] double LineProjectionParameter(const SCPoint2d& point,
                                                     const SCLineSegment2d& segment,
                                                     const SCVector2d& direction,
                                                     double length)
        {
            const double lengthSquared = direction.LengthSquared();
            if (std::isnormal(lengthSquared))
            {
                const double parameter = Geometry::Dot(point - segment.startPoint, direction) / lengthSquared;
                if (std::isfinite(parameter))
                {
                    return parameter;
                }
            }
            const SCVector2d unit{direction.x / length, direction.y / length};
            return Geometry::Dot(point - segment.startPoint, unit) / length;
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
        const double rawParameter = LineProjectionParameter(point, segment, direction, segment.Length());
        const double parameter = clampToSegment ? Clamp01(rawParameter) : rawParameter;
        const SCPoint2d projectedPoint = segment.PointAt(parameter);

        return SCSegmentProjection2d{projectedPoint,
                                   parameter,
                                   DistanceSquared(point, projectedPoint),
                                   rawParameter >= -Geometry::kDefaultParameterEpsilon &&
                                       rawParameter <= 1.0 + Geometry::kDefaultParameterEpsilon};
    }

    SCSegmentProjection2d ProjectPointToLineSegment(const SCPoint2d& point,
                                                  const SCLineSegment2d& segment,
                                                  bool clampToSegment,
                                                  double epsilon)
    {
        if (!(epsilon > 0.0) || !std::isfinite(epsilon) || !point.IsValid() ||
            !segment.startPoint.IsValid() || !segment.endPoint.IsValid())
        {
            return InvalidSegmentProjection2d();
        }
        if (!segment.IsValid(epsilon))
        {
            return {segment.startPoint, 0.0, DistanceSquared(point, segment.startPoint), true};
        }

        const SCVector2d direction = segment.endPoint - segment.startPoint;
        const double length = std::hypot(direction.x, direction.y);
        if (!std::isfinite(length) || !(length > 0.0))
        {
            return InvalidSegmentProjection2d();
        }

        const double rawParameter = LineProjectionParameter(point, segment, direction, length);
        if (!std::isfinite(rawParameter))
        {
            return InvalidSegmentProjection2d();
        }
        const double parameter = clampToSegment ? Clamp01(rawParameter) : rawParameter;
        const SCPoint2d projectedPoint{std::lerp(segment.startPoint.x, segment.endPoint.x, parameter),
                                       std::lerp(segment.startPoint.y, segment.endPoint.y, parameter)};
        const double distanceSquared = DistanceSquared(point, projectedPoint);
        const double parameterTolerance = epsilon / length;
        const SCSegmentProjection2d projection{projectedPoint,
                                                parameter,
                                                distanceSquared,
                                                rawParameter >= -parameterTolerance &&
                                                    rawParameter <= 1.0 + parameterTolerance};
        return projection.IsValid() ? projection : InvalidSegmentProjection2d();
    }

    SCSegmentProjection2d ProjectPointToSegment(const SCPoint2d& point,
                                              const SCPoint2d& segmentStart,
                                              const SCPoint2d& segmentEnd,
                                              bool clampToSegment)
    {
        if ((segmentEnd - segmentStart).LengthSquared() <=
            Geometry::kDefaultEpsilon * Geometry::kDefaultEpsilon)
        {
            return {segmentStart, 0.0, DistanceSquared(point, segmentStart), true};
        }

        const SCLineSegment2d segment = SCLineSegment2d::FromEndpoints(segmentStart, segmentEnd);
        SCSegmentProjection2d projection = ProjectPointToLineSegment(point, segment, clampToSegment);
        projection.isOnSegment = clampToSegment || projection.isOnSegment;
        return projection;
    }

    SCSegmentProjection2d ProjectPointToSegment(const SCPoint2d& point,
                                              const SCPoint2d& segmentStart,
                                              const SCPoint2d& segmentEnd,
                                              bool clampToSegment,
                                              double epsilon)
    {
        SCSegmentProjection2d projection = ProjectPointToLineSegment(
            point, SCLineSegment2d::FromEndpoints(segmentStart, segmentEnd), clampToSegment, epsilon);
        if (projection.IsValid())
        {
            projection.isOnSegment = clampToSegment || projection.isOnSegment;
        }
        return projection;
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
        const double parameter = clampToSegment ? ClampedArcProjectionParameter(point, segment, rawParameter)
                                                : rawParameter;
        const SCPoint2d projectedPoint = segment.PointAt(parameter);

        return SCSegmentProjection2d{projectedPoint,
                                   parameter,
                                   DistanceSquared(point, projectedPoint),
                                   rawParameter >= -Geometry::kDefaultParameterEpsilon &&
                                       rawParameter <= 1.0 + Geometry::kDefaultParameterEpsilon};
    }

    SCSegmentProjection2d ProjectPointToArcSegment(const SCPoint2d& point,
                                                 const SCArcSegment2d& segment,
                                                 bool clampToSegment,
                                                 double epsilon)
    {
        if (!(epsilon > 0.0) || !std::isfinite(epsilon) || !point.IsValid() || !segment.IsValid(epsilon))
        {
            return InvalidSegmentProjection2d();
        }
        const double length = segment.Length(epsilon);
        const SCVector2d radial = point - segment.center;
        if (!(length > 0.0) || !std::isfinite(length) ||
            !std::isfinite(radial.x) || !std::isfinite(radial.y))
        {
            return InvalidSegmentProjection2d();
        }

        const double rawParameter = ParameterOnArc(segment, std::atan2(radial.y, radial.x));
        if (!std::isfinite(rawParameter))
        {
            return InvalidSegmentProjection2d();
        }

        const double parameter = clampToSegment ? ClampedArcProjectionParameter(point, segment, rawParameter)
                                                : rawParameter;
        const SCPoint2d projectedPoint = segment.PointAt(parameter, epsilon);
        const double revolutionParameter = 2.0 * std::numbers::pi_v<double> / std::abs(segment.sweepAngle);
        const double parameterTolerance = epsilon / length;
        const SCSegmentProjection2d projection{
            projectedPoint,
            parameter,
            DistanceSquared(point, projectedPoint),
            rawParameter <= 1.0 + parameterTolerance ||
                rawParameter >= revolutionParameter - parameterTolerance};
        return projection.IsValid() ? projection : InvalidSegmentProjection2d();
    }

    SCSegmentProjection2d ProjectPointToSegment(const SCPoint2d& point, const ISCSegment2d& segment, bool clampToSegment)
    {
        if (segment.Kind() == SCSegmentKind2d::Line)
        {
            return ProjectPointToLineSegment(point, static_cast<const SCLineSegment2d&>(segment), clampToSegment);
        }

        return ProjectPointToArcSegment(point, static_cast<const SCArcSegment2d&>(segment), clampToSegment);
    }

    SCSegmentProjection2d ProjectPointToSegment(const SCPoint2d& point,
                                              const ISCSegment2d& segment,
                                              bool clampToSegment,
                                              double epsilon)
    {
        if (segment.Kind() == SCSegmentKind2d::Line)
        {
            return ProjectPointToLineSegment(point, static_cast<const SCLineSegment2d&>(segment),
                                             clampToSegment, epsilon);
        }
        if (segment.Kind() == SCSegmentKind2d::Arc)
        {
            return ProjectPointToArcSegment(point, static_cast<const SCArcSegment2d&>(segment),
                                            clampToSegment, epsilon);
        }
        return InvalidSegmentProjection2d();
    }

    double ParameterAtLength(const ISCSegment2d& segment, double length, bool clampToSegment)
    {
        const double totalLength = segment.Length();
        if (totalLength <= Geometry::kDefaultEpsilon)
        {
            return 0.0;
        }

        if (clampToSegment)
        {
            length = std::max(0.0, std::min(length, totalLength));
        }
        return length / totalLength;
    }

    double ParameterAtLength(const ISCSegment2d& segment, double length, bool clampToSegment, double epsilon)
    {
        if (!std::isfinite(epsilon) || epsilon <= 0.0 || !std::isfinite(length))
        {
            return 0.0;
        }

        double totalLength = 0.0;
        if (const auto* arc = dynamic_cast<const SCArcSegment2d*>(&segment))
        {
            totalLength = arc->Length(epsilon);
        }
        else if (const auto* line = dynamic_cast<const SCLineSegment2d*>(&segment))
        {
            totalLength = line->IsValid(epsilon) ? line->Length() : 0.0;
        }
        else
        {
            totalLength = segment.Length();
        }
        if (!std::isfinite(totalLength) || totalLength <= epsilon)
        {
            return 0.0;
        }

        if (clampToSegment)
        {
            length = std::clamp(length, 0.0, totalLength);
        }
        const double parameter = length / totalLength;
        return std::isfinite(parameter) ? parameter : 0.0;
    }

    double ParameterAtLength(const SCLineSegment2d& segment, double length, bool clampToSegment)
    {
        return ParameterAtLength(static_cast<const ISCSegment2d&>(segment), length, clampToSegment);
    }

    double ParameterAtLength(const SCArcSegment2d& segment, double length, bool clampToSegment)
    {
        return ParameterAtLength(static_cast<const ISCSegment2d&>(segment), length, clampToSegment);
    }

    SCVector2d TangentAt(const SCLineSegment2d& segment, double)
    {
        const SCVector2d direction = segment.endPoint - segment.startPoint;
        const double length = direction.Length();
        if (length <= Geometry::kDefaultEpsilon)
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
        if (radialLength <= Geometry::kDefaultEpsilon)
        {
            return SCVector2d{};
        }
        radial = radial / radialLength;
        return segment.Direction() == SCArcDirection2d::CounterClockwise ? SCVector2d{-radial.y, radial.x}
                                                                     : SCVector2d{radial.y, -radial.x};
    }

    SCVector2d TangentAt(const ISCSegment2d& segment, double parameter)
    {
        if (segment.Kind() == SCSegmentKind2d::Line)
        {
            return TangentAt(static_cast<const SCLineSegment2d&>(segment), parameter);
        }

        return TangentAt(static_cast<const SCArcSegment2d&>(segment), parameter);
    }

    SCVector2d TangentAt(const SCLineSegment2d& segment, double parameter, double epsilon)
    {
        if (!std::isfinite(epsilon) || epsilon <= 0.0 || !std::isfinite(parameter) ||
            !segment.IsValid(epsilon))
        {
            return {};
        }
        const SCVector2d direction = segment.endPoint - segment.startPoint;
        const double length = direction.Length();
        if (!std::isfinite(length) || length <= epsilon)
        {
            return {};
        }
        const SCVector2d tangent = direction / length;
        return tangent.IsValid() ? tangent : SCVector2d{};
    }

    SCVector2d TangentAt(const SCArcSegment2d& segment, double parameter, double epsilon)
    {
        if (!std::isfinite(epsilon) || epsilon <= 0.0 || !std::isfinite(parameter) ||
            !segment.IsValid(epsilon))
        {
            return {};
        }
        const double arcLength = segment.Length(epsilon);
        if (!std::isfinite(arcLength) || arcLength <= epsilon)
        {
            return {};
        }
        const double angle = segment.startAngle + segment.sweepAngle * parameter;
        if (!std::isfinite(angle))
        {
            return {};
        }
        const SCVector2d unitRadial{std::cos(angle), std::sin(angle)};
        if (!unitRadial.IsValid())
        {
            return {};
        }
        return segment.Direction() == SCArcDirection2d::CounterClockwise
                   ? SCVector2d{-unitRadial.y, unitRadial.x}
                   : SCVector2d{unitRadial.y, -unitRadial.x};
    }

    SCVector2d TangentAt(const ISCSegment2d& segment, double parameter, double epsilon)
    {
        if (segment.Kind() == SCSegmentKind2d::Line)
        {
            return TangentAt(static_cast<const SCLineSegment2d&>(segment), parameter, epsilon);
        }
        if (segment.Kind() == SCSegmentKind2d::Arc)
        {
            return TangentAt(static_cast<const SCArcSegment2d&>(segment), parameter, epsilon);
        }
        return {};
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

    SCVector2d NormalAt(const SCLineSegment2d& segment, double parameter, double epsilon)
    {
        return LeftNormal(TangentAt(segment, parameter, epsilon));
    }

    SCVector2d NormalAt(const SCArcSegment2d& segment, double parameter, double epsilon)
    {
        return LeftNormal(TangentAt(segment, parameter, epsilon));
    }

    SCVector2d NormalAt(const ISCSegment2d& segment, double parameter, double epsilon)
    {
        return LeftNormal(TangentAt(segment, parameter, epsilon));
    }

    SCSegmentProjection3d ProjectPointToLineSegment(const SCPoint3d& point,
                                                    const SCLineSegment3d& segment,
                                                    bool clampToSegment,
                                                    const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !point.IsValid() ||
            !segment.startPoint.IsValid() || !segment.endPoint.IsValid())
        {
            return InvalidSegmentProjection3d();
        }

        const SCVector3d direction = segment.endPoint - segment.startPoint;
        const double length = std::hypot(direction.x, direction.y, direction.z);
        if (!std::isfinite(length))
        {
            return InvalidSegmentProjection3d();
        }
        if (length <= tolerance.distanceEpsilon)
        {
            const SCSegmentProjection3d projection{segment.startPoint, 0.0,
                                                    (point - segment.startPoint).LengthSquared(), true};
            return projection.IsValid() ? projection : InvalidSegmentProjection3d();
        }

        const SCVector3d unitDirection = direction / length;
        const double rawParameter = Dot(point - segment.startPoint, unitDirection) / length;
        if (!std::isfinite(rawParameter))
        {
            return InvalidSegmentProjection3d();
        }
        const double parameter = clampToSegment ? std::clamp(rawParameter, 0.0, 1.0) : rawParameter;
        const SCPoint3d projectedPoint = segment.PointAt(parameter);
        const SCSegmentProjection3d projection{projectedPoint,
                                                parameter,
                                                (point - projectedPoint).LengthSquared(),
                                                rawParameter >= -tolerance.parameterEpsilon &&
                                                    rawParameter <= 1.0 + tolerance.parameterEpsilon};
        return projection.IsValid() ? projection : InvalidSegmentProjection3d();
    }

    SCSegmentProjection3d ProjectPointToLineSegment(const SCPoint3d& point,
                                                    const SCPoint3d& segmentStart,
                                                    const SCPoint3d& segmentEnd,
                                                    bool clampToSegment,
                                                    const SCGeometryTolerance3d& tolerance)
    {
        return ProjectPointToLineSegment(point,
                                         SCLineSegment3d{segmentStart, segmentEnd},
                                         clampToSegment,
                                         tolerance);
    }
}  // namespace Geometry
