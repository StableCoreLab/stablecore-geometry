#include "Core/AxisOps.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>
#include <utility>

#include "Core/Algorithms.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Support/Epsilon.h"
#include "Support/Geometry2d/Predicate2.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] double Clamp01(double value)
        {
            return std::clamp(value, 0.0, 1.0);
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

        [[nodiscard]] SCPoint2d Interpolate(const SCPoint2d& startPoint, const SCPoint2d& endPoint, double parameter)
        {
            return SCPoint2d{startPoint.x + (endPoint.x - startPoint.x) * parameter,
                           startPoint.y + (endPoint.y - startPoint.y) * parameter};
        }

        [[nodiscard]] SCSegmentProjection2d ProjectPointToLineSegmentImpl(const SCPoint2d& point,
                                                                        const SCLineSegment2d& segment,
                                                                        bool clampToSegment)
        {
            if (!segment.IsValid())
            {
                return SCSegmentProjection2d{segment.startPoint, 0.0, DistanceSquared(point, segment.startPoint), true};
            }

            const SCVector2d direction = segment.endPoint - segment.startPoint;
            const double lengthSquared = direction.LengthSquared();
            if (lengthSquared <= Geometry::kAxisOpsDefaultEpsilon * Geometry::kAxisOpsDefaultEpsilon)
            {
                return SCSegmentProjection2d{segment.startPoint, 0.0, DistanceSquared(point, segment.startPoint), true};
            }

            const double rawParameter = Dot(point - segment.startPoint, direction) / lengthSquared;
            const double parameter = clampToSegment ? Clamp01(rawParameter) : rawParameter;
            const SCPoint2d projectedPoint = Interpolate(segment.startPoint, segment.endPoint, parameter);
            return SCSegmentProjection2d{projectedPoint,
                                       parameter,
                                       DistanceSquared(point, projectedPoint),
                                       clampToSegment || (rawParameter >= -Geometry::kAxisOpsDefaultEpsilon &&
                                                          rawParameter <= 1.0 + Geometry::kAxisOpsDefaultEpsilon)};
        }

        [[nodiscard]] double ParameterOnArc(const SCArcSegment2d& segment, double angle)
        {
            if (segment.sweepAngle >= 0.0)
            {
                return NormalizeAngle(angle - segment.startAngle) / segment.sweepAngle;
            }

            return NormalizeAngle(segment.startAngle - angle) / (-segment.sweepAngle);
        }

        [[nodiscard]] SCPoint2d PointAtAngle(const SCArcSegment2d& segment, double angle)
        {
            return SCPoint2d{segment.center.x + segment.radius * std::cos(angle),
                           segment.center.y + segment.radius * std::sin(angle)};
        }

        [[nodiscard]] SCSegmentProjection2d ProjectPointToArcSegmentImpl(const SCPoint2d& point,
                                                                       const SCArcSegment2d& segment,
                                                                       bool clampToSegment)
        {
            if (!segment.IsValid())
            {
                return SCSegmentProjection2d{SCPoint2d{}, 0.0, -1.0, false};
            }

            const SCVector2d radial = point - segment.center;
            const double radialLengthSquared = radial.LengthSquared();
            if (radialLengthSquared <= Geometry::kAxisOpsDefaultEpsilon * Geometry::kAxisOpsDefaultEpsilon)
            {
                const SCPoint2d startPoint = segment.StartPoint();
                return SCSegmentProjection2d{startPoint, 0.0, DistanceSquared(point, startPoint), clampToSegment};
            }

            const double angle = std::atan2(radial.y, radial.x);
            const double rawParameter = ParameterOnArc(segment, angle);
            const double parameter = clampToSegment ? Clamp01(rawParameter) : rawParameter;
            const SCPoint2d projectedPoint = segment.PointAt(parameter);
            return SCSegmentProjection2d{projectedPoint,
                                       parameter,
                                       DistanceSquared(point, projectedPoint),
                                       clampToSegment || (rawParameter >= -Geometry::kAxisOpsDefaultEpsilon &&
                                                          rawParameter <= 1.0 + Geometry::kAxisOpsDefaultEpsilon)};
        }

        [[nodiscard]] SCVector2d UnitTangentAt(const SCLineSegment2d& segment)
        {
            const SCVector2d direction = segment.endPoint - segment.startPoint;
            const double length = direction.Length();
            if (length <= Geometry::kAxisOpsDefaultEpsilon)
            {
                return SCVector2d{};
            }

            return direction / length;
        }

        [[nodiscard]] SCVector2d UnitTangentAt(const SCArcSegment2d& segment, double parameter)
        {
            if (!segment.IsValid())
            {
                return SCVector2d{};
            }

            const SCPoint2d point = segment.PointAt(parameter);
            SCVector2d radial = point - segment.center;
            const double radialLength = radial.Length();
            if (radialLength <= Geometry::kAxisOpsDefaultEpsilon)
            {
                return SCVector2d{};
            }

            radial = radial / radialLength;
            if (segment.sweepAngle >= 0.0)
            {
                return SCVector2d{-radial.y, radial.x};
            }

            return SCVector2d{radial.y, -radial.x};
        }

        [[nodiscard]] SCVector2d LeftNormal(const SCVector2d& tangent)
        {
            return SCVector2d{-tangent.y, tangent.x};
        }
    }  // namespace

    SCAxisSample2d SampleAxis(const ISCSegment2d& segment, double parameter)
    {
        const SCPoint2d point = segment.PointAt(parameter);
        const SCVector2d tangent = segment.Kind() == SCSegmentKind2::Line
                                     ? UnitTangentAt(static_cast<const SCLineSegment2d&>(segment))
                                     : UnitTangentAt(static_cast<const SCArcSegment2d&>(segment), parameter);
        const SCVector2d normal = LeftNormal(tangent);
        return SCAxisSample2d{point, tangent, normal, parameter};
    }

    SCAxisSample2d SampleAxisAtLength(const ISCSegment2d& segment, double length, bool clampToSegment)
    {
        const double totalLength = segment.Length();
        if (totalLength <= Geometry::kAxisOpsDefaultEpsilon)
        {
            return SampleAxis(segment, 0.0);
        }

        if (clampToSegment)
        {
            length = std::clamp(length, 0.0, totalLength);
        }

        return SampleAxis(segment, length / totalLength);
    }

    SCAxisProjection2d ProjectPointToAxis(const SCPoint2d& point, const ISCSegment2d& segment)
    {
        if (segment.Kind() == SCSegmentKind2::Line)
        {
            const SCLineSegment2d& line = static_cast<const SCLineSegment2d&>(segment);
            const SCSegmentProjection2d projection = ProjectPointToLineSegmentImpl(point, line, true);
            const SCVector2d tangent = UnitTangentAt(line);
            return SCAxisProjection2d{projection, tangent, LeftNormal(tangent)};
        }

        const SCArcSegment2d& arc = static_cast<const SCArcSegment2d&>(segment);
        const SCSegmentProjection2d projection = ProjectPointToArcSegmentImpl(point, arc, true);
        const SCVector2d tangent = UnitTangentAt(arc, projection.parameter);
        return SCAxisProjection2d{projection, tangent, LeftNormal(tangent)};
    }

    std::unique_ptr<ISCSegment2d> Reverse(const ISCSegment2d& segment)
    {
        if (segment.Kind() == SCSegmentKind2::Line)
        {
            const SCLineSegment2d& line = static_cast<const SCLineSegment2d&>(segment);
            return std::make_unique<SCLineSegment2d>(line.endPoint, line.startPoint);
        }

        const SCArcSegment2d& arc = static_cast<const SCArcSegment2d&>(segment);
        return std::make_unique<SCArcSegment2d>(arc.center, arc.radius, arc.EndAngle(), -arc.sweepAngle);
    }

    SCSegmentSplit2d SplitSegment(const ISCSegment2d& segment, double parameter)
    {
        SCSegmentSplit2d result;
        if (!segment.IsValid() || !std::isfinite(parameter) || parameter <= 0.0 || parameter >= 1.0)
        {
            return result;
        }

        if (segment.Kind() == SCSegmentKind2::Line)
        {
            const SCLineSegment2d& line = static_cast<const SCLineSegment2d&>(segment);
            const SCPoint2d splitPoint = line.PointAt(parameter);
            result.first = std::make_unique<SCLineSegment2d>(line.startPoint, splitPoint);
            result.second = std::make_unique<SCLineSegment2d>(splitPoint, line.endPoint);
        } else
        {
            const SCArcSegment2d& arc = static_cast<const SCArcSegment2d&>(segment);
            const double firstSweep = arc.sweepAngle * parameter;
            result.first = std::make_unique<SCArcSegment2d>(arc.center, arc.radius, arc.startAngle, firstSweep);
            result.second = std::make_unique<SCArcSegment2d>(
                arc.center, arc.radius, arc.startAngle + firstSweep, arc.sweepAngle - firstSweep);
        }

        result.success =
            result.first != nullptr && result.second != nullptr && result.first->IsValid() && result.second->IsValid();
        return result;
    }

    SCSegmentTrim2d TrimSegment(const ISCSegment2d& segment, double startParameter, double endParameter)
    {
        SCSegmentTrim2d result;
        if (!segment.IsValid() || !std::isfinite(startParameter) || !std::isfinite(endParameter))
        {
            return result;
        }

        if (endParameter < startParameter)
        {
            std::swap(startParameter, endParameter);
        }

        startParameter = std::clamp(startParameter, 0.0, 1.0);
        endParameter = std::clamp(endParameter, 0.0, 1.0);
        if (endParameter - startParameter <= Geometry::kAxisOpsDefaultEpsilon)
        {
            return result;
        }

        if (segment.Kind() == SCSegmentKind2::Line)
        {
            const SCLineSegment2d& line = static_cast<const SCLineSegment2d&>(segment);
            result.segment = std::make_unique<SCLineSegment2d>(line.PointAt(startParameter), line.PointAt(endParameter));
        } else
        {
            const SCArcSegment2d& arc = static_cast<const SCArcSegment2d&>(segment);
            result.segment = std::make_unique<SCArcSegment2d>(arc.center,
                                                            arc.radius,
                                                            arc.startAngle + arc.sweepAngle * startParameter,
                                                            arc.sweepAngle * (endParameter - startParameter));
        }

        result.success = result.segment != nullptr && result.segment->IsValid();
        return result;
    }

    SCSnapResult2d SnapPointToSegments(const SCPoint2d& point,
                                     std::span<const ISCSegment2d* const> segments,
                                     double maxDistance)
    {
        SCSnapResult2d result;
        if (!(maxDistance >= 0.0))
        {
            return result;
        }

        double bestDistanceSquared = maxDistance * maxDistance;
        for (std::size_t i = 0; i < segments.size(); ++i)
        {
            const ISCSegment2d* segment = segments[i];
            if (segment == nullptr || !segment->IsValid())
            {
                continue;
            }

            const SCSegmentProjection2d projection = ProjectPointToAxis(point, *segment).projection;
            if (!projection.IsValid() || projection.distanceSquared > bestDistanceSquared)
            {
                continue;
            }

            result.snapped = true;
            result.point = projection.point;
            result.distanceSquared = projection.distanceSquared;
            result.segmentIndex = i;
            result.parameter = projection.parameter;
            bestDistanceSquared = projection.distanceSquared;
        }

        return result;
    }
}  // namespace Geometry

