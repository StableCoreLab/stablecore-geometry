#include "Core/Offset.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <memory>
#include <numbers>
#include <optional>
#include <utility>
#include <vector>

#include "../Detail/PathNumerics3d.h"
#include "Support/Geometry2d/Predicate2.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] double Cross2(const SCVector2d& a, const SCVector2d& b)
        { return a.x * b.y - a.y * b.x; }

        [[nodiscard]] double Distance2(const SCPoint2d& a, const SCPoint2d& b)
        { return PathDetail::Norm(a - b); }

        [[nodiscard]] double SegmentLength(const ISCSegment2d& segment)
        {
            if (segment.Kind() == SCSegmentKind2::Line)
            {
                const auto& line = static_cast<const SCLineSegment2d&>(segment);
                return Distance2(line.startPoint, line.endPoint);
            }
            const auto& arc = static_cast<const SCArcSegment2d&>(segment);
            return arc.radius * std::abs(arc.sweepAngle);
        }

        [[nodiscard]] bool FiniteSegment(const ISCSegment2d& segment)
        {
            if (segment.Kind() == SCSegmentKind2::Line)
            {
                const auto* line = dynamic_cast<const SCLineSegment2d*>(&segment);
                return line && line->startPoint.IsValid() && line->endPoint.IsValid();
            }
            if (segment.Kind() == SCSegmentKind2::Arc)
            {
                const auto* arc = dynamic_cast<const SCArcSegment2d*>(&segment);
                return arc && arc->center.IsValid() && std::isfinite(arc->radius) &&
                       std::isfinite(arc->startAngle) && std::isfinite(arc->sweepAngle);
            }
            return false;
        }

        [[nodiscard]] SCPathGeometryFailure GeometricValidity(
            const ISCSegment2d& segment, const SCPathGeometryTolerance& tolerance)
        {
            if (segment.Kind() == SCSegmentKind2::Line)
            {
                const auto& line = static_cast<const SCLineSegment2d&>(segment);
                return Distance2(line.startPoint, line.endPoint) > tolerance.distanceTolerance
                    ? SCPathGeometryFailure::None : SCPathGeometryFailure::DegenerateDirection;
            }
            if (segment.Kind() == SCSegmentKind2::Arc)
            {
                const auto& arc = static_cast<const SCArcSegment2d&>(segment);
                return arc.radius > tolerance.distanceTolerance &&
                       std::abs(arc.sweepAngle) > tolerance.angleToleranceRad &&
                       std::abs(arc.sweepAngle) <= 2.0 * std::numbers::pi_v<double>
                    ? SCPathGeometryFailure::None : SCPathGeometryFailure::DegenerateDirection;
            }
            return SCPathGeometryFailure::UnsupportedSegment;
        }

        struct PairHits
        {
            bool overlap{false};
            bool numericalFailure{false};
            std::vector<SCPoint2d> points{};

            void Add(const SCPoint2d& point)
            {
                if (!point.IsValid()) { numericalFailure = true; return; }
                for (const auto& existing : points)
                {
                    const double scale = std::max({std::abs(existing.x), std::abs(existing.y),
                                                   std::abs(point.x), std::abs(point.y)});
                    if (Distance2(existing, point) <=
                        PathDetail::kPathRepresentationTolerance * scale)
                    { return; }
                }
                points.push_back(point);
            }
        };

        [[nodiscard]] std::optional<double> CircleHeight(double radius, double offset)
        {
            const double ratio = std::abs(offset) / radius;
            if (!std::isfinite(ratio) || ratio > 1.0) { return {}; }
            return radius * std::sqrt((1.0 - ratio) * (1.0 + ratio));
        }

        [[nodiscard]] double CircleIntersectionDistance(double firstRadius, double secondRadius,
                                                          double centerDistance)
        {
            const double scale = std::max({firstRadius, secondRadius, centerDistance});
            const double a = firstRadius / scale, b = secondRadius / scale;
            const double d = centerDistance / scale;
            return scale * (((a - b) * (a + b) + d * d) / (2.0 * d));
        }

        [[nodiscard]] bool OnArc(const SCArcSegment2d& arc, const SCPoint2d& point,
                                 double tolerance, bool interior = false)
        {
            const double distance = Distance2(arc.center, point);
            if (std::abs(distance - arc.radius) > tolerance) { return false; }
            const double angle = std::atan2(point.y - arc.center.y, point.x - arc.center.x);
            const double start = std::remainder(arc.startAngle, 2.0 * std::numbers::pi_v<double>);
            const double phase = std::remainder(angle - start, 2.0 * std::numbers::pi_v<double>);
            const double margin = (tolerance / arc.radius) / std::abs(arc.sweepAngle);
            for (int turn = -2; turn <= 2; ++turn)
            {
                const double parameter = (phase + turn * 2.0 * std::numbers::pi_v<double>) /
                                         arc.sweepAngle;
                if (interior ? parameter > margin && parameter < 1.0 - margin
                             : parameter >= -margin && parameter <= 1.0 + margin)
                { return true; }
            }
            return false;
        }

        [[nodiscard]] PairHits LineLineHits(const SCLineSegment2d& first,
                                            const SCLineSegment2d& second, double tolerance)
        {
            PairHits hits;
            const SCVector2d a = first.endPoint - first.startPoint;
            const SCVector2d b = second.endPoint - second.startPoint;
            const double aLength = PathDetail::Norm(a), bLength = PathDetail::Norm(b);
            const SCVector2d u = a / aLength, v = b / bLength;
            const SCVector2d delta = second.startPoint - first.startPoint;
            const double divisor = Cross2(u, v);
            if (!delta.IsValid() || !std::isfinite(divisor))
            { hits.numericalFailure = true; return hits; }
            if (std::abs(divisor) <= PathDetail::kPathRepresentationTolerance)
            {
                if (std::abs(Cross2(delta, u)) > tolerance) { return hits; }
                const double p = delta.x * u.x + delta.y * u.y;
                const double q = p + b.x * u.x + b.y * u.y;
                if (!std::isfinite(p) || !std::isfinite(q))
                { hits.numericalFailure = true; return hits; }
                const double lo = std::max(0.0, std::min(p, q));
                const double hi = std::min(aLength, std::max(p, q));
                if (lo > hi && lo - hi > tolerance) { return hits; }
                if (hi - lo > PathDetail::kPathRepresentationTolerance * aLength)
                { hits.overlap = true; return hits; }
                hits.Add(first.startPoint + u * std::clamp(lo, 0.0, aLength));
                return hits;
            }
            const double alongA = Cross2(delta, v) / divisor;
            const double alongB = Cross2(delta, u) / divisor;
            if (alongA >= -tolerance && alongA <= aLength + tolerance &&
                alongB >= -tolerance && alongB <= bLength + tolerance)
            { hits.Add(first.startPoint + u * alongA); }
            return hits;
        }

        [[nodiscard]] PairHits LineArcHits(const SCLineSegment2d& line,
                                           const SCArcSegment2d& arc, double tolerance)
        {
            PairHits hits;
            const SCVector2d direction = line.endPoint - line.startPoint;
            const double length = PathDetail::Norm(direction);
            const SCVector2d u = direction / length;
            const SCVector2d fromStart = arc.center - line.startPoint;
            const double projection = fromStart.x * u.x + fromStart.y * u.y;
            const double offAxis = Cross2(fromStart, u);
            if (!std::isfinite(projection) || !std::isfinite(offAxis))
            { hits.numericalFailure = true; return hits; }
            const auto offset = CircleHeight(arc.radius, offAxis);
            if (!offset) { return hits; }
            for (const double position : {projection - *offset, projection + *offset})
            {
                if (position < -tolerance || position > length + tolerance) { continue; }
                const SCPoint2d point = line.startPoint + u * position;
                if (!point.IsValid())
                { hits.numericalFailure = true; return hits; }
                if (OnArc(arc, point, tolerance)) { hits.Add(point); }
            }
            return hits;
        }

        [[nodiscard]] PairHits ArcArcHits(const SCArcSegment2d& first,
                                          const SCArcSegment2d& second, double tolerance)
        {
            PairHits hits;
            const SCVector2d separation = second.center - first.center;
            const double distance = PathDetail::Norm(separation);
            if (!std::isfinite(distance))
            { hits.numericalFailure = true; return hits; }
            if (distance == 0.0 && first.radius == second.radius)
            {
                for (const SCPoint2d& endpoint : {first.StartPoint(), first.EndPoint()})
                {
                    if (OnArc(second, endpoint, tolerance, true))
                    { hits.overlap = true; return hits; }
                    if (OnArc(second, endpoint, tolerance)) { hits.Add(endpoint); }
                }
                for (const SCPoint2d& endpoint : {second.StartPoint(), second.EndPoint()})
                {
                    if (OnArc(first, endpoint, tolerance, true))
                    { hits.overlap = true; return hits; }
                    if (OnArc(first, endpoint, tolerance)) { hits.Add(endpoint); }
                }
                if (OnArc(second, first.PointAt(0.5), tolerance, true) ||
                    OnArc(first, second.PointAt(0.5), tolerance, true))
                { hits.overlap = true; }
                if (std::abs(first.sweepAngle) == 2.0 * std::numbers::pi_v<double> ||
                    std::abs(second.sweepAngle) == 2.0 * std::numbers::pi_v<double>)
                { hits.overlap = true; }
                return hits;
            }
            if (distance == 0.0 || distance > first.radius + second.radius ||
                distance < std::abs(first.radius - second.radius))
            { return hits; }
            const double along = CircleIntersectionDistance(first.radius, second.radius, distance);
            if (!std::isfinite(along)) { hits.numericalFailure = true; return hits; }
            const auto height = CircleHeight(first.radius, along);
            if (!height) { return hits; }
            const SCVector2d u = separation / distance;
            const SCPoint2d foot = first.center + u * along;
            const SCVector2d perpendicular{-u.y, u.x};
            for (const SCPoint2d& point : {foot + perpendicular * *height,
                                           foot - perpendicular * *height})
            {
                if (!point.IsValid())
                { hits.numericalFailure = true; return hits; }
                if (OnArc(first, point, tolerance) && OnArc(second, point, tolerance))
                { hits.Add(point); }
            }
            return hits;
        }

        [[nodiscard]] PairHits SegmentHits(const ISCSegment2d& first,
                                           const ISCSegment2d& second, double tolerance)
        {
            if (first.Kind() == SCSegmentKind2::Line && second.Kind() == SCSegmentKind2::Line)
            { return LineLineHits(static_cast<const SCLineSegment2d&>(first),
                                  static_cast<const SCLineSegment2d&>(second), tolerance); }
            if (first.Kind() == SCSegmentKind2::Line)
            { return LineArcHits(static_cast<const SCLineSegment2d&>(first),
                                 static_cast<const SCArcSegment2d&>(second), tolerance); }
            if (second.Kind() == SCSegmentKind2::Line)
            { return LineArcHits(static_cast<const SCLineSegment2d&>(second),
                                 static_cast<const SCArcSegment2d&>(first), tolerance); }
            return ArcArcHits(static_cast<const SCArcSegment2d&>(first),
                              static_cast<const SCArcSegment2d&>(second), tolerance);
        }

        [[nodiscard]] PairHits SupportHits(const ISCSegment2d& first,
                                           const ISCSegment2d& second, double tolerance)
        {
            PairHits hits;
            if (first.Kind() == SCSegmentKind2::Line && second.Kind() == SCSegmentKind2::Line)
            {
                const auto& a = static_cast<const SCLineSegment2d&>(first);
                const auto& b = static_cast<const SCLineSegment2d&>(second);
                const SCVector2d au = (a.endPoint - a.startPoint) /
                                       PathDetail::Norm(a.endPoint - a.startPoint);
                const SCVector2d bu = (b.endPoint - b.startPoint) /
                                       PathDetail::Norm(b.endPoint - b.startPoint);
                const SCVector2d delta = b.startPoint - a.startPoint;
                const double divisor = Cross2(au, bu);
                if (!std::isfinite(divisor) || !delta.IsValid())
                { hits.numericalFailure = true; return hits; }
                if (std::abs(divisor) <= PathDetail::kPathRepresentationTolerance)
                { hits.overlap = std::abs(Cross2(delta, au)) <= tolerance; return hits; }
                hits.Add(a.startPoint + au * (Cross2(delta, bu) / divisor));
                return hits;
            }
            if (first.Kind() == SCSegmentKind2::Line || second.Kind() == SCSegmentKind2::Line)
            {
                const auto& line = static_cast<const SCLineSegment2d&>(
                    first.Kind() == SCSegmentKind2::Line ? first : second);
                const auto& arc = static_cast<const SCArcSegment2d&>(
                    first.Kind() == SCSegmentKind2::Arc ? first : second);
                const SCVector2d u = (line.endPoint - line.startPoint) /
                                      PathDetail::Norm(line.endPoint - line.startPoint);
                const SCVector2d delta = arc.center - line.startPoint;
                const double along = delta.x * u.x + delta.y * u.y;
                const double offAxis = Cross2(delta, u);
                if (!std::isfinite(along) || !std::isfinite(offAxis))
                { hits.numericalFailure = true; return hits; }
                const auto height = CircleHeight(arc.radius, offAxis);
                if (!height) { return hits; }
                hits.Add(line.startPoint + u * (along - *height));
                hits.Add(line.startPoint + u * (along + *height));
                return hits;
            }
            const auto& a = static_cast<const SCArcSegment2d&>(first);
            const auto& b = static_cast<const SCArcSegment2d&>(second);
            const SCVector2d delta = b.center - a.center;
            const double distance = PathDetail::Norm(delta);
            if (!std::isfinite(distance))
            { hits.numericalFailure = true; return hits; }
            if (distance == 0.0 && a.radius == b.radius)
            { hits.overlap = true; return hits; }
            if (distance == 0.0 || distance > a.radius + b.radius ||
                distance < std::abs(a.radius - b.radius))
            { return hits; }
            const double along = CircleIntersectionDistance(a.radius, b.radius, distance);
            if (!std::isfinite(along)) { hits.numericalFailure = true; return hits; }
            const auto height = CircleHeight(a.radius, along);
            if (!height) { return hits; }
            const SCVector2d u = delta / distance;
            const SCVector2d perpendicular{-u.y, u.x};
            const SCPoint2d foot = a.center + u * along;
            hits.Add(foot + perpendicular * *height);
            hits.Add(foot - perpendicular * *height);
            return hits;
        }

        struct TopologyFailure
        {
            bool invalid{false};
            std::size_t first{0};
            std::size_t second{0};
            SCPathGeometryFailure failure{SCPathGeometryFailure::SelfIntersection};
        };

        [[nodiscard]] TopologyFailure CheckTopology(
            const std::vector<std::shared_ptr<ISCSegment2d>>& segments, bool closed,
            double tolerance)
        {
            const std::size_t count = segments.size();
            if (count == 1)
            {
                if (closed)
                {
                    const auto* arc = dynamic_cast<const SCArcSegment2d*>(segments[0].get());
                    if (!arc || std::abs(arc->sweepAngle) != 2.0 * std::numbers::pi_v<double>)
                    { return {true, 0, 0}; }
                }
                else if (Distance2(segments[0]->StartPoint(), segments[0]->EndPoint()) <= tolerance)
                { return {true, 0, 0}; }
                return {};
            }
            for (std::size_t i = 0; i < count; ++i)
            {
                const auto* arc = dynamic_cast<const SCArcSegment2d*>(segments[i].get());
                if (arc && Distance2(arc->StartPoint(), arc->EndPoint()) <= tolerance)
                { return {true, i, i}; }
                for (std::size_t j = i + 1; j < count; ++j)
                {
                    const bool sequential = j == i + 1;
                    const bool seam = closed && i == 0 && j + 1 == count;
                    const auto intersection = SegmentHits(*segments[i], *segments[j], tolerance);
                    if (intersection.numericalFailure)
                    { return {true, i, j, SCPathGeometryFailure::NonFiniteResult}; }
                    if (!intersection.overlap && intersection.points.empty()) { continue; }
                    if ((!sequential && !seam) || intersection.overlap)
                    { return {true, i, j}; }
                    bool seenSequential = false;
                    bool seenSeam = false;
                    for (const auto& point : intersection.points)
                    {
                        const bool atSequential = sequential &&
                            Distance2(point, segments[i]->EndPoint()) <= tolerance &&
                            Distance2(point, segments[j]->StartPoint()) <= tolerance;
                        const bool atSeam = seam &&
                            Distance2(point, segments[j]->EndPoint()) <= tolerance &&
                            Distance2(point, segments[i]->StartPoint()) <= tolerance;
                        if (atSequential && !seenSequential) { seenSequential = true; }
                        else if (atSeam && !seenSeam) { seenSeam = true; }
                        else { return {true, i, j}; }
                    }
                }
            }
            return {};
        }

        struct ParameterChoice
        {
            double value{0.0};
            bool ambiguous{false};
        };

        [[nodiscard]] ParameterChoice OffsetParameter(const ISCSegment2d& support,
                                                       const SCPoint2d& point, double reference,
                                                       bool beforeJoin)
        {
            if (support.Kind() == SCSegmentKind2::Line)
            {
                const auto& line = static_cast<const SCLineSegment2d&>(support);
                const SCVector2d direction = line.endPoint - line.startPoint;
                const double length = PathDetail::Norm(direction);
                const SCVector2d displacement = point - line.startPoint;
                const SCVector2d unit = direction / length;
                return {(displacement.x * unit.x + displacement.y * unit.y) / length};
            }
            const auto& arc = static_cast<const SCArcSegment2d&>(support);
            const SCVector2d radial = point - arc.center;
            const double principal = std::atan2(radial.y, radial.x);
            const double start = std::remainder(arc.startAngle, 2.0 * std::numbers::pi_v<double>);
            const double phase = std::remainder(principal - start, 2.0 * std::numbers::pi_v<double>);
            const double target = reference * arc.sweepAngle;
            const double period = 2.0 * std::numbers::pi_v<double>;
            const double rawTurns = (target - phase) / period;
            const double turns = std::round(rawTurns);
            const double parameter = (phase + turns * period) / arc.sweepAngle;
            const double lower = std::floor(rawTurns);
            if (std::abs(rawTurns - lower - 0.5) > PathDetail::kPathRepresentationTolerance)
            { return {parameter}; }
            const double lowerParameter = (phase + lower * period) / arc.sweepAngle;
            const double upperParameter = (phase + (lower + 1.0) * period) / arc.sweepAngle;
            const double sweep = std::abs(arc.sweepAngle);
            const auto valid = [beforeJoin, sweep, period](double value)
            {
                return beforeJoin ? value > 0.0 && value * sweep <= period
                                  : value < 1.0 && (1.0 - value) * sweep <= period;
            };
            const bool lowerValid = valid(lowerParameter);
            const bool upperValid = valid(upperParameter);
            if (lowerValid && upperValid) { return {0.0, true}; }
            if (lowerValid) { return {lowerParameter}; }
            if (upperValid) { return {upperParameter}; }
            return {parameter};
        }

        [[nodiscard]] std::shared_ptr<ISCSegment2d> TrimSupport(
            const ISCSegment2d& support, double start, double end)
        {
            if (support.Kind() == SCSegmentKind2::Line)
            {
                const auto& line = static_cast<const SCLineSegment2d&>(support);
                const SCVector2d direction = line.endPoint - line.startPoint;
                return std::make_shared<SCLineSegment2d>(line.startPoint + direction * start,
                                                         line.startPoint + direction * end);
            }
            const auto& arc = static_cast<const SCArcSegment2d&>(support);
            return std::make_shared<SCArcSegment2d>(arc.center, arc.radius,
                                                     arc.startAngle + arc.sweepAngle * start,
                                                     arc.sweepAngle * (end - start));
        }

        struct JoinResult
        {
            SCPathGeometryFailure failure{SCPathGeometryFailure::JoinNotFound};
            double beforeEnd{0.0};
            double afterStart{0.0};
        };

        [[nodiscard]] JoinResult FindJoin(const ISCSegment2d& before, const ISCSegment2d& after,
                                          const SCPoint2d& originalVertex, double signedDistance,
                                          double miterLimit, double tolerance)
        {
            const double limit = std::abs(signedDistance) * miterLimit + tolerance;
            if (!std::isfinite(limit))
            { return {SCPathGeometryFailure::NonFiniteResult}; }
            const auto candidates = SupportHits(before, after, tolerance);
            if (candidates.numericalFailure)
            { return {SCPathGeometryFailure::NonFiniteResult}; }
            if (candidates.overlap)
            {
                const double endpointGap = Distance2(before.EndPoint(), after.StartPoint());
                if (!std::isfinite(endpointGap))
                { return {SCPathGeometryFailure::NonFiniteResult}; }
                if (endpointGap <= tolerance)
                {
                    return {SCPathGeometryFailure::None, 1.0, 0.0};
                }
                return {SCPathGeometryFailure::AmbiguousJoin};
            }
            JoinResult selected;
            bool exceededMiter = false;
            std::size_t accepted = 0;
            for (const SCPoint2d& point : candidates.points)
            {
                if (!point.IsValid()) { return {SCPathGeometryFailure::NonFiniteResult}; }
                const auto beforeChoice = OffsetParameter(before, point, 1.0, true);
                const auto afterChoice = OffsetParameter(after, point, 0.0, false);
                if (beforeChoice.ambiguous || afterChoice.ambiguous)
                { return {SCPathGeometryFailure::AmbiguousJoin}; }
                const double beforeParameter = beforeChoice.value;
                const double afterParameter = afterChoice.value;
                if (!std::isfinite(beforeParameter) || !std::isfinite(afterParameter))
                { return {SCPathGeometryFailure::NonFiniteResult}; }
                if (beforeParameter <= 0.0 || afterParameter >= 1.0)
                { continue; }
                if (before.Kind() == SCSegmentKind2::Arc &&
                    beforeParameter * std::abs(static_cast<const SCArcSegment2d&>(before).sweepAngle) >
                        2.0 * std::numbers::pi_v<double>)
                { continue; }
                if (after.Kind() == SCSegmentKind2::Arc &&
                    (1.0 - afterParameter) *
                        std::abs(static_cast<const SCArcSegment2d&>(after).sweepAngle) >
                        2.0 * std::numbers::pi_v<double>)
                { continue; }
                const double distance = Distance2(point, originalVertex);
                if (!std::isfinite(distance))
                { return {SCPathGeometryFailure::NonFiniteResult}; }
                if (distance > limit)
                { exceededMiter = true; continue; }
                ++accepted;
                selected = {SCPathGeometryFailure::None, beforeParameter, afterParameter};
            }
            if (accepted > 1) { return {SCPathGeometryFailure::AmbiguousJoin}; }
            if (accepted == 1) { return selected; }
            return {exceededMiter ? SCPathGeometryFailure::MiterLimitExceeded
                                  : SCPathGeometryFailure::JoinNotFound};
        }
    }

    SCMappedOffsetResult2d OffsetWithMapping(const SCPolyline2d& polyline, double signedDistance,
                                              const SCMappedOffsetOptions2d& options,
                                              const SCPathGeometryTolerance& tolerance)
    {
        SCMappedOffsetResult2d result;
        const auto fail = [](SCPathGeometryFailure reason, std::optional<std::size_t> first = {},
                             std::optional<std::size_t> second = {})
        {
            SCMappedOffsetResult2d failed;
            failed.failure = reason;
            failed.inputIndex = first;
            failed.relatedIndex = second;
            return failed;
        };
        if (!tolerance.IsValid()) { return fail(SCPathGeometryFailure::InvalidTolerance); }
        if (!std::isfinite(signedDistance)) { return fail(SCPathGeometryFailure::NonFiniteInput); }
        if (!std::isfinite(options.miterLimit) || options.miterLimit < 1.0)
        { return fail(SCPathGeometryFailure::InvalidOptions); }
        const std::size_t count = polyline.SegmentCount();
        if (count == 0) { return fail(SCPathGeometryFailure::InsufficientPoints); }
        std::vector<std::shared_ptr<ISCSegment2d>> sources;
        sources.reserve(count);
        for (std::size_t i = 0; i < count; ++i)
        {
            std::unique_ptr<ISCSegment2d> segment = polyline.SegmentAt(i);
            if (!segment) { return fail(SCPathGeometryFailure::UnsupportedSegment, i); }
            if ((segment->Kind() == SCSegmentKind2::Line &&
                 !dynamic_cast<const SCLineSegment2d*>(segment.get())) ||
                (segment->Kind() == SCSegmentKind2::Arc &&
                 !dynamic_cast<const SCArcSegment2d*>(segment.get())))
            { return fail(SCPathGeometryFailure::UnsupportedSegment, i); }
            if (!FiniteSegment(*segment))
            { return fail(segment->Kind() == SCSegmentKind2::Line ||
                          segment->Kind() == SCSegmentKind2::Arc
                              ? SCPathGeometryFailure::NonFiniteInput
                              : SCPathGeometryFailure::UnsupportedSegment, i); }
            sources.emplace_back(std::move(segment));
        }
        PathDetail::CompensatedSum sourceLength;
        for (std::size_t i = 0; i < count; ++i)
        {
            const auto& segment = sources[i];
            const auto validity = GeometricValidity(*segment, tolerance);
            if (validity != SCPathGeometryFailure::None) { return fail(validity, i); }
            if (!segment->StartPoint().IsValid() || !segment->EndPoint().IsValid())
            { return fail(SCPathGeometryFailure::NonFiniteResult, i); }
            if (!segment->IsValid()) { return fail(SCPathGeometryFailure::Unrepresentable2d, i); }
            const double length = SegmentLength(*segment);
            if (!std::isfinite(length)) { return fail(SCPathGeometryFailure::NonFiniteResult, i); }
            if (length == 0.0) { return fail(SCPathGeometryFailure::NumericalFailure, i); }
            sourceLength.Add(length);
            if (!std::isfinite(sourceLength.Value()))
            { return fail(SCPathGeometryFailure::NonFiniteResult, i); }
            if (i > 0)
            {
                const double gap = Distance2(sources[i - 1]->EndPoint(), sources[i]->StartPoint());
                if (!std::isfinite(gap))
                { return fail(SCPathGeometryFailure::NonFiniteResult, i - 1, i); }
                if (gap > tolerance.distanceTolerance)
                { return fail(SCPathGeometryFailure::DiscontinuousPath, i - 1, i); }
            }
        }
        if (polyline.IsClosed())
        {
            const double gap = Distance2(sources.back()->EndPoint(), sources.front()->StartPoint());
            if (!std::isfinite(gap))
            { return fail(SCPathGeometryFailure::NonFiniteResult, count - 1, 0); }
            if (gap > tolerance.distanceTolerance)
            { return fail(SCPathGeometryFailure::DiscontinuousPath, count - 1, 0); }
        }
        for (std::size_t i = 1; i < count; ++i)
        {
            if (!IsEqual(sources[i - 1]->EndPoint(), sources[i]->StartPoint()))
            { return fail(SCPathGeometryFailure::Unrepresentable2d, i - 1, i); }
        }
        if (polyline.IsClosed() &&
            !IsEqual(sources.back()->EndPoint(), sources.front()->StartPoint()))
        { return fail(SCPathGeometryFailure::Unrepresentable2d, count - 1, 0); }
        if (!polyline.IsValid()) { return fail(SCPathGeometryFailure::Unrepresentable2d); }
        const auto inputTopology = CheckTopology(sources, polyline.IsClosed(), tolerance.distanceTolerance);
        if (inputTopology.invalid)
        { return fail(inputTopology.failure, inputTopology.first, inputTopology.second); }
        std::vector<std::shared_ptr<ISCSegment2d>> supports;
        supports.reserve(count);
        for (std::size_t i = 0; i < count; ++i)
        {
            if (signedDistance == 0.0)
            { supports.emplace_back(sources[i]->Clone()); continue; }
            if (sources[i]->Kind() == SCSegmentKind2::Line)
            {
                const auto& line = static_cast<const SCLineSegment2d&>(*sources[i]);
                const SCVector2d direction = line.endPoint - line.startPoint;
                const double length = PathDetail::Norm(direction);
                const SCVector2d left{-direction.y / length, direction.x / length};
                supports.push_back(std::make_shared<SCLineSegment2d>(
                    line.startPoint + left * signedDistance, line.endPoint + left * signedDistance));
            }
            else
            {
                const auto& arc = static_cast<const SCArcSegment2d&>(*sources[i]);
                const double radius = arc.radius -
                    (arc.sweepAngle > 0.0 ? signedDistance : -signedDistance);
                if (radius <= tolerance.distanceTolerance)
                { return fail(SCPathGeometryFailure::DegenerateOffset, i); }
                supports.push_back(std::make_shared<SCArcSegment2d>(
                    arc.center, radius, arc.startAngle, arc.sweepAngle));
            }
            if (!supports.back()->StartPoint().IsValid() || !supports.back()->EndPoint().IsValid())
            { return fail(SCPathGeometryFailure::NonFiniteResult, i); }
            if (GeometricValidity(*supports.back(), tolerance) != SCPathGeometryFailure::None)
            { return fail(SCPathGeometryFailure::DegenerateOffset, i); }
            const double supportLength = SegmentLength(*supports.back());
            if (!std::isfinite(supportLength))
            { return fail(SCPathGeometryFailure::NonFiniteResult, i); }
            if (supportLength == 0.0)
            { return fail(SCPathGeometryFailure::DegenerateOffset, i); }
            if (!supports.back()->IsValid())
            { return fail(SCPathGeometryFailure::Unrepresentable2d, i); }
        }
        std::vector<double> starts(count, 0.0), ends(count, 1.0);
        if (signedDistance != 0.0 && count > 1)
        {
            const std::size_t joins = polyline.IsClosed() ? count : count - 1;
            for (std::size_t i = 0; i < joins; ++i)
            {
                const std::size_t next = (i + 1) % count;
                const JoinResult join = FindJoin(*supports[i], *supports[next], sources[i]->EndPoint(),
                                                 signedDistance, options.miterLimit,
                                                 tolerance.distanceTolerance);
                if (join.failure != SCPathGeometryFailure::None)
                { return fail(join.failure, i, next); }
                ends[i] = join.beforeEnd;
                starts[next] = join.afterStart;
            }
        }
        std::vector<std::shared_ptr<ISCSegment2d>> output;
        output.reserve(count);
        PathDetail::CompensatedSum outputLength;
        result.outputCumulativeLengths.push_back(0.0);
        for (std::size_t i = 0; i < count; ++i)
        {
            if (ends[i] <= starts[i] ||
                (supports[i]->Kind() == SCSegmentKind2::Arc &&
                 (ends[i] - starts[i]) *
                     std::abs(static_cast<const SCArcSegment2d&>(*supports[i]).sweepAngle) >
                     2.0 * std::numbers::pi_v<double>))
            { return fail(SCPathGeometryFailure::DegenerateOffset, i); }
            auto segment = TrimSupport(*supports[i], starts[i], ends[i]);
            const auto validity = GeometricValidity(*segment, tolerance);
            if (validity != SCPathGeometryFailure::None)
            { return fail(SCPathGeometryFailure::DegenerateOffset, i); }
            if (!segment->StartPoint().IsValid() || !segment->EndPoint().IsValid())
            { return fail(SCPathGeometryFailure::NonFiniteResult, i); }
            if (!segment->IsValid()) { return fail(SCPathGeometryFailure::Unrepresentable2d, i); }
            const double length = SegmentLength(*segment);
            if (!std::isfinite(length)) { return fail(SCPathGeometryFailure::NonFiniteResult, i); }
            if (length == 0.0) { return fail(SCPathGeometryFailure::DegenerateOffset, i); }
            outputLength.Add(length);
            if (!std::isfinite(outputLength.Value()))
            { return fail(SCPathGeometryFailure::NonFiniteResult, i); }
            result.mappings.push_back({i, i, starts[i], ends[i], SegmentLength(*sources[i]), length});
            result.outputCumulativeLengths.push_back(outputLength.Value());
            output.push_back(std::move(segment));
        }
        for (std::size_t i = 1; i < count; ++i)
        {
            const double gap = Distance2(output[i - 1]->EndPoint(), output[i]->StartPoint());
            if (!std::isfinite(gap))
            { return fail(SCPathGeometryFailure::NonFiniteResult, i - 1, i); }
            if (gap > tolerance.distanceTolerance)
            { return fail(SCPathGeometryFailure::Unrepresentable2d, i - 1, i); }
            if (!IsEqual(output[i - 1]->EndPoint(), output[i]->StartPoint()))
            { return fail(SCPathGeometryFailure::Unrepresentable2d, i - 1, i); }
        }
        if (polyline.IsClosed())
        {
            const double gap = Distance2(output.back()->EndPoint(), output.front()->StartPoint());
            if (!std::isfinite(gap))
            { return fail(SCPathGeometryFailure::NonFiniteResult, count - 1, 0); }
            if (gap > tolerance.distanceTolerance)
            { return fail(SCPathGeometryFailure::Unrepresentable2d, count - 1, 0); }
        }
        if (polyline.IsClosed() &&
            !IsEqual(output.back()->EndPoint(), output.front()->StartPoint()))
        { return fail(SCPathGeometryFailure::Unrepresentable2d, count - 1, 0); }
        const auto outputTopology = CheckTopology(output, polyline.IsClosed(), tolerance.distanceTolerance);
        if (outputTopology.invalid)
        { return fail(outputTopology.failure, outputTopology.first, outputTopology.second); }
        SCPolyline2d value(std::move(output), polyline.IsClosed() ? SCPolylineClosure::Closed
                                                                  : SCPolylineClosure::Open);
        if (!value.IsValid()) { return fail(SCPathGeometryFailure::Unrepresentable2d); }
        result.polyline = std::move(value);
        result.inputTotalLength2d = sourceLength.Value();
        result.outputTotalLength2d = outputLength.Value();
        result.success = true;
        return result;
    }
}
