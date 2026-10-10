#include "Core/PathGeometry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <numbers>

#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "../Detail/PathNumerics3d.h"

namespace Geometry
{
    namespace
    {
        using PathDetail::Norm;
        using PathDetail::CompensatedSum;

        struct Witness
        {
            SCVector3d normal{};
            SCPathGeometryFailure failure{SCPathGeometryFailure::RankDeficient};
        };

        [[nodiscard]] Witness OrderedWitness(std::span<const SCPoint3d> points,
                                             const SCPathGeometryTolerance& tolerance)
        {
            for (std::size_t i = 0; i + 2 < points.size(); ++i)
            {
                for (std::size_t j = i + 1; j + 1 < points.size(); ++j)
                {
                    for (std::size_t k = j + 1; k < points.size(); ++k)
                    {
                        const std::array<std::size_t, 3> indices{i, j, k};
                        if (Norm(points[j] - points[i]) <= tolerance.distanceTolerance ||
                            Norm(points[k] - points[j]) <= tolerance.distanceTolerance ||
                            Norm(points[i] - points[k]) <= tolerance.distanceTolerance)
                        {
                            continue;
                        }
                        double bestAngle = -1.0;
                        double bestHeight = 0.0;
                        SCVector3d bestNormal{};
                        for (int turn = 0; turn < 3; ++turn)
                        {
                            const auto a = indices[turn];
                            const auto b = indices[(turn + 1) % 3];
                            const auto c = indices[(turn + 2) % 3];
                            const SCVector3d first = points[b] - points[a];
                            const SCVector3d second = points[c] - points[a];
                            const double firstLength = Norm(first);
                            const double secondLength = Norm(second);
                            if (!std::isfinite(firstLength) || !std::isfinite(secondLength))
                            {
                                return {{}, SCPathGeometryFailure::NonFiniteResult};
                            }
                            const SCVector3d unitFirst = first / firstLength;
                            const SCVector3d unitSecond = second / secondLength;
                            const SCVector3d cross = Cross(unitFirst, unitSecond);
                            const double angle = Norm(cross);
                            const double height = Norm(Cross(unitFirst, second));
                            if (!std::isfinite(angle) || !std::isfinite(height))
                            {
                                return {{}, SCPathGeometryFailure::NonFiniteResult};
                            }
                            if (angle > bestAngle)
                            {
                                bestAngle = angle;
                                bestHeight = height;
                                bestNormal = cross;
                            }
                        }
                        if (bestHeight <= tolerance.distanceTolerance)
                        {
                            continue;
                        }
                        if (bestAngle <= PathDetail::kPathRepresentationTolerance)
                        {
                            return {{}, SCPathGeometryFailure::NumericalFailure};
                        }
                        return {bestNormal / bestAngle, SCPathGeometryFailure::None};
                    }
                }
            }
            return {};
        }

        [[nodiscard]] bool FinitePoints(std::span<const SCPoint3d> points,
                                         std::optional<std::size_t>& firstBad)
        {
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                if (!points[i].IsValid())
                {
                    firstBad = i;
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] SCVector3d SvdNormal(std::vector<std::array<double, 3>>& rows,
                                            double relativeTolerance,
                                            SCPathGeometryFailure& failure)
        {
            std::array<std::array<double, 3>, 3> rotations{{{1,0,0}, {0,1,0}, {0,0,1}}};
            std::array<double, 3> lengths{};
            bool converged = false;
            for (int sweep = 0; sweep < 32; ++sweep)
            {
                converged = true;
                for (int p = 0; p < 2; ++p)
                {
                    for (int q = p + 1; q < 3; ++q)
                    {
                        CompensatedSum pp, qq, pq;
                        double scaleP = 0.0, scaleQ = 0.0;
                        for (const auto& row : rows)
                        {
                            scaleP = std::max(scaleP, std::abs(row[p]));
                            scaleQ = std::max(scaleQ, std::abs(row[q]));
                        }
                        if (scaleP == 0.0 || scaleQ == 0.0)
                        {
                            continue;
                        }
                        for (const auto& row : rows)
                        {
                            const double a = row[p] / scaleP;
                            const double b = row[q] / scaleQ;
                            pp.Add(a * a);
                            qq.Add(b * b);
                            pq.Add(a * b);
                        }
                        const double np = std::sqrt(pp.Value());
                        const double nq = std::sqrt(qq.Value());
                        const double correlation = pq.Value() / (np * nq);
                        if (!std::isfinite(correlation))
                        {
                            failure = SCPathGeometryFailure::NumericalFailure;
                            return {};
                        }
                        if (std::abs(correlation) <= PathDetail::kPathRepresentationTolerance)
                        {
                            continue;
                        }
                        converged = false;
                        const double normP = scaleP * np;
                        const double normQ = scaleQ * nq;
                        const double dot = correlation * normP * normQ;
                        const double angle = 0.5 * std::atan2(2.0 * dot,
                                                              normQ * normQ - normP * normP);
                        const double c = std::cos(angle), s = std::sin(angle);
                        for (auto& row : rows)
                        {
                            const double oldP = row[p], oldQ = row[q];
                            row[p] = c * oldP - s * oldQ;
                            row[q] = s * oldP + c * oldQ;
                        }
                        for (auto& row : rotations)
                        {
                            const double oldP = row[p], oldQ = row[q];
                            row[p] = c * oldP - s * oldQ;
                            row[q] = s * oldP + c * oldQ;
                        }
                    }
                }
                if (converged)
                {
                    break;
                }
            }
            if (!converged)
            {
                converged = true;
                for (int p = 0; p < 2; ++p)
                {
                    for (int q = p + 1; q < 3; ++q)
                    {
                        double scaleP = 0.0, scaleQ = 0.0;
                        for (const auto& row : rows)
                        {
                            scaleP = std::max(scaleP, std::abs(row[p]));
                            scaleQ = std::max(scaleQ, std::abs(row[q]));
                        }
                        if (scaleP == 0.0 || scaleQ == 0.0) { continue; }
                        CompensatedSum pp, qq, pq;
                        for (const auto& row : rows)
                        {
                            const double a = row[p] / scaleP, b = row[q] / scaleQ;
                            pp.Add(a * a);
                            qq.Add(b * b);
                            pq.Add(a * b);
                        }
                        const double correlation = pq.Value() /
                            (std::sqrt(pp.Value()) * std::sqrt(qq.Value()));
                        if (!std::isfinite(correlation) ||
                            std::abs(correlation) > PathDetail::kPathRepresentationTolerance)
                        { converged = false; }
                    }
                }
                if (!converged)
                {
                    failure = SCPathGeometryFailure::NumericalFailure;
                    return {};
                }
            }
            for (int col = 0; col < 3; ++col)
            {
                double norm = 0.0;
                for (const auto& row : rows)
                {
                    norm = std::hypot(norm, row[col]);
                }
                lengths[col] = norm;
            }
            std::array<int, 3> order{0,1,2};
            std::stable_sort(order.begin(), order.end(),
                             [&](int a, int b) { return lengths[a] < lengths[b]; });
            const double sigma0 = lengths[order[0]], sigma1 = lengths[order[1]];
            const double sigma2 = lengths[order[2]];
            if (!std::isfinite(sigma2) || sigma1 <= PathDetail::kPathRepresentationTolerance * sigma2)
            {
                failure = SCPathGeometryFailure::NumericalFailure;
                return {};
            }
            if (sigma1 - sigma0 <= relativeTolerance * sigma1)
            {
                failure = SCPathGeometryFailure::AmbiguousPlane;
                return {};
            }
            const SCVector3d normal(rotations[0][order[0]], rotations[1][order[0]],
                                    rotations[2][order[0]]);
            failure = SCPathGeometryFailure::None;
            return normal;
        }
    }

    bool SCPathGeometryTolerance::IsValid() const
    {
        return std::isfinite(distanceTolerance) && distanceTolerance > 0.0 &&
               std::isfinite(angleToleranceRad) && angleToleranceRad > 0.0 &&
               angleToleranceRad < std::numbers::pi_v<double> / 2.0 &&
               std::isfinite(relativeTolerance) && relativeTolerance > 0.0 &&
               relativeTolerance < 1.0;
    }

    SCPlaneFitResult FitPlane(std::span<const SCPoint3d> orderedPoints,
                             const SCPathGeometryTolerance& tolerance)
    {
        SCPlaneFitResult result;
        if (!tolerance.IsValid()) { result.failure = SCPathGeometryFailure::InvalidTolerance; return result; }
        if (!FinitePoints(orderedPoints, result.inputIndex))
        { result.failure = SCPathGeometryFailure::NonFiniteInput; return result; }
        if (orderedPoints.size() < 3)
        { result.failure = SCPathGeometryFailure::InsufficientPoints; return result; }
        double extent = 0.0;
        for (const auto& point : orderedPoints)
        {
            const SCVector3d difference = point - orderedPoints.front();
            if (!difference.IsValid())
            { result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
            extent = std::max({extent, std::abs(difference.x),
                               std::abs(difference.y), std::abs(difference.z)});
        }
        if (extent <= tolerance.distanceTolerance)
        { result.failure = SCPathGeometryFailure::CoincidentPoints; return result; }
        const Witness witness = OrderedWitness(orderedPoints, tolerance);
        if (witness.failure != SCPathGeometryFailure::None)
        { result.failure = witness.failure; return result; }
        std::array<CompensatedSum, 3> totals{};
        std::vector<std::array<double, 3>> rows;
        rows.reserve(orderedPoints.size());
        for (const auto& point : orderedPoints)
        {
            const SCVector3d difference = point - orderedPoints.front();
            const std::array<double, 3> row{difference.x / extent, difference.y / extent,
                                             difference.z / extent};
            rows.push_back(row);
            for (int axis = 0; axis < 3; ++axis) { totals[axis].Add(row[axis]); }
        }
        std::array<double, 3> mean{};
        for (int axis = 0; axis < 3; ++axis)
        {
            mean[axis] = totals[axis].Value() / static_cast<double>(rows.size());
        }
        for (auto& row : rows)
        {
            for (int axis = 0; axis < 3; ++axis) { row[axis] -= mean[axis]; }
        }
        SCPathGeometryFailure failure;
        SCVector3d normal = SvdNormal(rows, tolerance.relativeTolerance, failure);
        if (failure != SCPathGeometryFailure::None)
        { result.failure = failure; return result; }
        const double alignment = Dot(normal, witness.normal);
        if (std::abs(alignment) <= std::sin(tolerance.angleToleranceRad))
        { result.failure = SCPathGeometryFailure::AmbiguousPlane; return result; }
        if (alignment < 0.0) { normal = -normal; }
        if (!PathDetail::Unit(normal, tolerance))
        { result.failure = SCPathGeometryFailure::NumericalFailure; return result; }
        const SCPoint3d origin = orderedPoints.front() +
            SCVector3d(mean[0] * extent, mean[1] * extent, mean[2] * extent);
        if (!origin.IsValid())
        { result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
        result.signedDistances.reserve(orderedPoints.size());
        for (const auto& point : orderedPoints)
        {
            const double distance = Dot(point - origin, normal);
            if (!std::isfinite(distance))
            { result = {}; result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
            result.signedDistances.push_back(distance);
            result.maxAbsoluteDeviation = std::max(result.maxAbsoluteDeviation, std::abs(distance));
        }
        result.plane = {origin, normal};
        result.success = true;
        return result;
    }

    SCPlaneFrameResult MakePlaneFrame(const SCPlane& plane,
                                     std::span<const SCPoint3d> orderedPoints,
                                     const SCPathGeometryTolerance& tolerance)
    {
        SCPlaneFrameResult result;
        if (!tolerance.IsValid()) { result.failure = SCPathGeometryFailure::InvalidTolerance; return result; }
        if (!FinitePoints(orderedPoints, result.inputIndex))
        { result.failure = SCPathGeometryFailure::NonFiniteInput; return result; }
        if (!plane.origin.IsValid() || !plane.normal.IsValid())
        { result.failure = SCPathGeometryFailure::InvalidPlane; return result; }
        if (orderedPoints.size() < 3)
        { result.failure = SCPathGeometryFailure::InsufficientPoints; return result; }
        const double scale = std::max({std::abs(plane.normal.x), std::abs(plane.normal.y),
                                       std::abs(plane.normal.z)});
        if (scale == 0.0)
        { result.failure = SCPathGeometryFailure::InvalidPlane; return result; }
        const SCVector3d scaledNormal = plane.normal / scale;
        const double length = Norm(scaledNormal);
        const Witness witness = OrderedWitness(orderedPoints, tolerance);
        if (witness.failure != SCPathGeometryFailure::None)
        { result.failure = witness.failure; return result; }
        SCVector3d n = scaledNormal / length;
        const double alignment = Dot(n, witness.normal);
        if (std::abs(alignment) <= std::sin(tolerance.angleToleranceRad))
        { result.failure = SCPathGeometryFailure::AmbiguousPlane; return result; }
        if (alignment < 0.0) { n = -n; }
        const double displacement = Dot(orderedPoints.front() - plane.origin, n);
        const SCPoint3d origin = orderedPoints.front() - n * displacement;
        if (!origin.IsValid())
        { result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
        for (std::size_t i = 1; i < orderedPoints.size(); ++i)
        {
            const SCVector3d delta = orderedPoints[i] - orderedPoints.front();
            const SCVector3d inPlane = delta - n * Dot(delta, n);
            const double size = Norm(inPlane);
            if (!std::isfinite(size))
            { result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
            if (size <= tolerance.distanceTolerance) { continue; }
            SCVector3d u = inPlane / size;
            SCVector3d v = Cross(n, u);
            const double vLength = Norm(v);
            if (!std::isfinite(vLength) || vLength == 0.0)
            { result.failure = SCPathGeometryFailure::NumericalFailure; return result; }
            v = v / vLength;
            u = Cross(v, n);
            result.frame = {origin, u, v, n};
            if (!PathDetail::ValidFrame(result.frame, tolerance))
            { result.frame = {}; result.failure = SCPathGeometryFailure::NumericalFailure; return result; }
            result.success = true;
            return result;
        }
        result.failure = SCPathGeometryFailure::DegenerateDirection;
        return result;
    }

    SCPlaneFrameResult MakeCollinearFrame(std::span<const SCPoint3d> orderedPoints,
                                         const SCPathGeometryTolerance& tolerance)
    {
        SCPlaneFrameResult result;
        if (!tolerance.IsValid()) { result.failure = SCPathGeometryFailure::InvalidTolerance; return result; }
        if (!FinitePoints(orderedPoints, result.inputIndex))
        { result.failure = SCPathGeometryFailure::NonFiniteInput; return result; }
        if (orderedPoints.size() < 2)
        { result.failure = SCPathGeometryFailure::InsufficientPoints; return result; }
        SCVector3d u;
        bool found = false;
        for (std::size_t i = 1; i < orderedPoints.size(); ++i)
        {
            const SCVector3d delta = orderedPoints[i] - orderedPoints.front();
            const double length = Norm(delta);
            if (!std::isfinite(length))
            { result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
            if (length > tolerance.distanceTolerance)
            {
                u = delta / length;
                found = true;
                break;
            }
        }
        if (!found) { result.failure = SCPathGeometryFailure::CoincidentPoints; return result; }
        for (std::size_t i = 0; i < orderedPoints.size(); ++i)
        {
            const SCVector3d delta = orderedPoints[i] - orderedPoints.front();
            const double distance = Norm(Cross(u, delta));
            if (!std::isfinite(distance))
            { result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
            if (distance > tolerance.distanceTolerance)
            { result.failure = SCPathGeometryFailure::NonCollinear; result.inputIndex = i; return result; }
        }
        const std::array<SCVector3d, 3> axes{SCVector3d{1,0,0},
                                             SCVector3d{0,1,0}, SCVector3d{0,0,1}};
        std::size_t best = 0;
        for (std::size_t i = 1; i < axes.size(); ++i)
        {
            if (std::abs(Dot(u, axes[i])) < std::abs(Dot(u, axes[best]))) { best = i; }
        }
        const SCVector3d projected = axes[best] - u * Dot(u, axes[best]);
        const double length = Norm(projected);
        if (!std::isfinite(length) || length == 0.0)
        { result.failure = SCPathGeometryFailure::NumericalFailure; return result; }
        SCVector3d v = projected / length;
        SCVector3d n = Cross(u, v);
        v = Cross(n, u);
        result.frame = {orderedPoints.front(), u, v, n};
        if (!PathDetail::ValidFrame(result.frame, tolerance))
        { result.frame = {}; result.failure = SCPathGeometryFailure::NumericalFailure; return result; }
        result.success = true;
        return result;
    }

    SCFrameProjectionResult ProjectToFrame(const SCPlaneFrame3d& frame,
                                           std::span<const SCPoint3d> points,
                                           const SCPathGeometryTolerance& tolerance)
    {
        SCFrameProjectionResult result;
        if (!tolerance.IsValid()) { result.failure = SCPathGeometryFailure::InvalidTolerance; return result; }
        if (!FinitePoints(points, result.inputIndex))
        { result.failure = SCPathGeometryFailure::NonFiniteInput; return result; }
        if (!PathDetail::ValidFrame(frame, tolerance))
        { result.failure = SCPathGeometryFailure::InvalidFrame; return result; }
        result.points.reserve(points.size());
        result.signedDistances.reserve(points.size());
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            const SCVector3d delta = points[i] - frame.origin;
            const double x = Dot(delta, frame.u), y = Dot(delta, frame.v);
            const double distance = Dot(delta, frame.n);
            const SCPoint3d lifted = frame.origin + frame.u * x + frame.v * y;
            const SCPoint3d projected = points[i] - frame.n * distance;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(distance) ||
                !lifted.IsValid() || !projected.IsValid())
            { result = {}; result.failure = SCPathGeometryFailure::NonFiniteResult; result.inputIndex = i; return result; }
            const double reconstructionError = Norm(lifted - projected);
            if (!std::isfinite(reconstructionError))
            { result = {}; result.failure = SCPathGeometryFailure::NonFiniteResult; result.inputIndex = i; return result; }
            if (reconstructionError > tolerance.distanceTolerance)
            { result = {}; result.failure = SCPathGeometryFailure::NumericalFailure; result.inputIndex = i; return result; }
            result.points.emplace_back(x, y);
            result.signedDistances.push_back(distance);
        }
        result.success = true;
        return result;
    }

    SCFrameLiftResult LiftFromFrame(const SCPlaneFrame3d& frame,
                                   std::span<const SCPoint2d> points,
                                   const SCPathGeometryTolerance& tolerance)
    {
        SCFrameLiftResult result;
        if (!tolerance.IsValid()) { result.failure = SCPathGeometryFailure::InvalidTolerance; return result; }
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            if (!points[i].IsValid())
            { result.failure = SCPathGeometryFailure::NonFiniteInput; result.inputIndex = i; return result; }
        }
        if (!PathDetail::ValidFrame(frame, tolerance))
        { result.failure = SCPathGeometryFailure::InvalidFrame; return result; }
        result.points.reserve(points.size());
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            const SCPoint3d lifted = frame.origin + frame.u * points[i].x + frame.v * points[i].y;
            if (!lifted.IsValid())
            { result = {}; result.failure = SCPathGeometryFailure::NonFiniteResult; result.inputIndex = i; return result; }
            const SCVector3d delta = lifted - frame.origin;
            const double x = Dot(delta, frame.u), y = Dot(delta, frame.v);
            const double distance = Dot(delta, frame.n);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(distance))
            { result = {}; result.failure = SCPathGeometryFailure::NonFiniteResult; result.inputIndex = i; return result; }
            const double reconstructionError = std::hypot(x - points[i].x, y - points[i].y);
            if (!std::isfinite(reconstructionError))
            { result = {}; result.failure = SCPathGeometryFailure::NonFiniteResult; result.inputIndex = i; return result; }
            if (reconstructionError > tolerance.distanceTolerance ||
                std::abs(distance) > tolerance.distanceTolerance)
            { result = {}; result.failure = SCPathGeometryFailure::NumericalFailure; result.inputIndex = i; return result; }
            result.points.push_back(lifted);
        }
        result.success = true;
        return result;
    }

    SCPolylineLengthResult MeasurePolyline3d(std::span<const SCPoint3d> points,
                                              SCPolylineClosure closure,
                                              const SCPathGeometryTolerance& tolerance)
    {
        SCPolylineLengthResult result;
        if (!tolerance.IsValid()) { result.failure = SCPathGeometryFailure::InvalidTolerance; return result; }
        if (!FinitePoints(points, result.inputIndex))
        { result.failure = SCPathGeometryFailure::NonFiniteInput; return result; }
        if (!PathDetail::ValidClosure(closure))
        { result.failure = SCPathGeometryFailure::InvalidClosure; return result; }
        if (points.size() < 2)
        { result.failure = SCPathGeometryFailure::InsufficientPoints; return result; }
        const std::size_t count = closure == SCPolylineClosure::Closed ? points.size() : points.size() - 1;
        CompensatedSum total;
        bool nonzero = false;
        result.cumulativeLengths.push_back(0.0);
        for (std::size_t i = 0; i < count; ++i)
        {
            const SCPoint3d& end = points[(i + 1) % points.size()];
            const double length = Norm(end - points[i]);
            if (!std::isfinite(length))
            { result = {}; result.failure = SCPathGeometryFailure::NonFiniteResult; result.inputIndex = i; return result; }
            if (length == 0.0 && points[i] != end)
            { result = {}; result.failure = SCPathGeometryFailure::NumericalFailure; result.inputIndex = i; return result; }
            nonzero |= !(points[i] == end);
            total.Add(length);
            if (!std::isfinite(total.Value()))
            { result = {}; result.failure = SCPathGeometryFailure::NonFiniteResult; result.inputIndex = i; return result; }
            result.segmentLengths.push_back(length);
            result.cumulativeLengths.push_back(total.Value());
        }
        if (!nonzero)
        { result = {}; result.failure = SCPathGeometryFailure::CoincidentPoints; return result; }
        result.totalLength = total.Value();
        result.success = true;
        return result;
    }

    namespace
    {
        [[nodiscard]] SCPoint3d SegmentStart(const SCPathSegment3d& segment)
        {
            if (const auto* line = std::get_if<SCLineSegment3d>(&segment)) { return line->startPoint; }
            return PathDetail::ArcPoint(std::get<SCArcSegment3d>(segment), 0.0);
        }

        [[nodiscard]] SCPoint3d SegmentEnd(const SCPathSegment3d& segment)
        {
            if (const auto* line = std::get_if<SCLineSegment3d>(&segment)) { return line->endPoint; }
            return PathDetail::ArcPoint(std::get<SCArcSegment3d>(segment), 1.0);
        }

        [[nodiscard]] bool Mergeable(std::span<const SCPathSegment3d> segments,
                                      const std::vector<std::size_t>& active,
                                      std::size_t begin, std::size_t end,
                                      const SCPathGeometryTolerance& tolerance)
        {
            const SCPoint3d start = SegmentStart(segments[active[begin]]);
            const SCPoint3d finish = SegmentEnd(segments[active[end]]);
            const SCVector3d chord = finish - start;
            const double size = Norm(chord);
            if (!std::isfinite(size) || size == 0.0) { return false; }
            const SCVector3d axis = chord / size;
            for (std::size_t position = begin; position <= end; ++position)
            {
                const auto* line = std::get_if<SCLineSegment3d>(&segments[active[position]]);
                if (!line) { return false; }
                const SCVector3d direction = line->endPoint - line->startPoint;
                const double length = Norm(direction);
                if (!(length > 0.0) || !std::isfinite(length)) { return false; }
                const double forward = Dot(direction, axis);
                const double angularDeviation = Norm(Cross(direction / length, axis));
                if (!std::isfinite(forward) || !std::isfinite(angularDeviation) ||
                    forward <= 0.0 ||
                    angularDeviation > std::sin(tolerance.angleToleranceRad))
                { return false; }
            }
            double previousParameter = 0.0;
            for (std::size_t source = active[begin]; source <= active[end]; ++source)
            {
                const auto* line = std::get_if<SCLineSegment3d>(&segments[source]);
                if (!line) { return false; }
                const SCVector3d direction = line->endPoint - line->startPoint;
                const double length = Norm(direction);
                if (!std::isfinite(length)) { return false; }
                if (line->startPoint == line->endPoint)
                {
                    for (const SCPoint3d& point : {line->startPoint, line->endPoint})
                    {
                        const SCVector3d offset = point - start;
                        const double parameter = Dot(offset, axis) / size;
                        const double deviation = Norm(Cross(offset, axis));
                        if (!std::isfinite(parameter) || !std::isfinite(deviation) ||
                            deviation > tolerance.distanceTolerance ||
                            parameter < 0.0 || parameter > 1.0 || parameter < previousParameter)
                        { return false; }
                        previousParameter = parameter;
                    }
                    continue;
                }
                for (const SCPoint3d& point : {line->startPoint, line->endPoint})
                {
                    const SCVector3d offset = point - start;
                    const double parameter = Dot(offset, axis) / size;
                    const double deviation = Norm(Cross(offset, axis));
                    if (!std::isfinite(parameter) || !std::isfinite(deviation) ||
                        deviation > tolerance.distanceTolerance ||
                        parameter < 0.0 || parameter > 1.0 || parameter < previousParameter)
                    { return false; }
                    previousParameter = parameter;
                }
            }
            return true;
        }

        struct SegmentGroup
        {
            std::size_t first{0};
            std::size_t last{0};
        };
    }

    SCPolylineSimplificationResult SimplifyPolylineWithMapping(
        std::span<const SCPathSegment3d> segments, const SCPathGeometryTolerance& tolerance,
        SCPolylineClosure closure)
    {
        SCPolylineSimplificationResult result;
        result.inputKind = SCPolylineSimplificationInputKind::SegmentSequence;
        const auto fail = [&](SCPathGeometryFailure failure, std::optional<std::size_t> first = {},
                              std::optional<std::size_t> second = {})
        {
            SCPolylineSimplificationResult failed;
            failed.inputKind = SCPolylineSimplificationInputKind::SegmentSequence;
            failed.failure = failure;
            failed.inputIndex = first;
            failed.relatedIndex = second;
            return failed;
        };
        if (!tolerance.IsValid()) { return fail(SCPathGeometryFailure::InvalidTolerance); }
        for (std::size_t i = 0; i < segments.size(); ++i)
        {
            if (const auto* line = std::get_if<SCLineSegment3d>(&segments[i]))
            {
                if (!line->startPoint.IsValid() || !line->endPoint.IsValid())
                { return fail(SCPathGeometryFailure::NonFiniteInput, i); }
            }
            else if (const auto* arc = std::get_if<SCArcSegment3d>(&segments[i]))
            {
                if (!arc->center.IsValid() || !arc->unitNormal.IsValid() ||
                    !arc->startRadialUnit.IsValid() || !std::isfinite(arc->radius) ||
                    !std::isfinite(arc->signedSweepRad))
                { return fail(SCPathGeometryFailure::NonFiniteInput, i); }
            }
        }
        if (!PathDetail::ValidClosure(closure)) { return fail(SCPathGeometryFailure::InvalidClosure); }
        if (segments.empty()) { return fail(SCPathGeometryFailure::InsufficientPoints); }
        result.sourceCumulativeLengths.push_back(0.0);
        CompensatedSum total;
        std::vector<std::size_t> active;
        for (std::size_t i = 0; i < segments.size(); ++i)
        {
            double length = 0.0;
            bool exactZero = false;
            if (const auto* line = std::get_if<SCLineSegment3d>(&segments[i]))
            {
                exactZero = line->startPoint == line->endPoint;
                length = Norm(line->endPoint - line->startPoint);
            }
            else
            {
                const auto& arc = std::get<SCArcSegment3d>(segments[i]);
                const auto failure = PathDetail::ValidateArc(arc, tolerance);
                if (failure != SCPathGeometryFailure::None) { return fail(failure, i); }
                length = arc.radius * std::abs(arc.signedSweepRad);
            }
            if (!std::isfinite(length)) { return fail(SCPathGeometryFailure::NonFiniteResult, i); }
            if (length == 0.0 && !exactZero)
            { return fail(SCPathGeometryFailure::NumericalFailure, i); }
            total.Add(length);
            if (!std::isfinite(total.Value())) { return fail(SCPathGeometryFailure::NonFiniteResult, i); }
            result.sourceSegmentLengths.push_back(length);
            result.sourceCumulativeLengths.push_back(total.Value());
            result.sourceMappings.push_back({i, {}, exactZero});
            if (!exactZero) { active.push_back(i); }
            if (i > 0)
            {
                const double gap = Norm(SegmentEnd(segments[i - 1]) - SegmentStart(segments[i]));
                if (!std::isfinite(gap))
                { return fail(SCPathGeometryFailure::NonFiniteResult, i - 1, i); }
                if (gap > tolerance.distanceTolerance)
                { return fail(SCPathGeometryFailure::DiscontinuousPath, i - 1, i); }
            }
        }
        if (closure == SCPolylineClosure::Closed)
        {
            const double gap = Norm(SegmentEnd(segments.back()) - SegmentStart(segments.front()));
            if (!std::isfinite(gap))
            { return fail(SCPathGeometryFailure::NonFiniteResult, segments.size() - 1, 0); }
            if (gap > tolerance.distanceTolerance)
            { return fail(SCPathGeometryFailure::DiscontinuousPath, segments.size() - 1, 0); }
        }
        if (active.empty()) { return fail(SCPathGeometryFailure::CoincidentPoints); }
        std::vector<SegmentGroup> groups;
        groups.reserve(active.size());
        for (std::size_t i = 0; i < active.size(); ++i) { groups.push_back({i, i}); }
        for (std::size_t i = 0; i < groups.size();)
        {
            const std::size_t source = active[groups[i].first];
            if (!std::holds_alternative<SCLineSegment3d>(segments[source]) ||
                result.sourceSegmentLengths[source] > tolerance.distanceTolerance)
            { ++i; continue; }
            if (i + 1 < groups.size() &&
                Mergeable(segments, active, groups[i].first, groups[i + 1].last, tolerance))
            {
                groups[i + 1].first = groups[i].first;
                groups.erase(groups.begin() + static_cast<std::ptrdiff_t>(i));
                continue;
            }
            if (i > 0 && Mergeable(segments, active, groups[i - 1].first, groups[i].last, tolerance))
            {
                groups[i - 1].last = groups[i].last;
                groups.erase(groups.begin() + static_cast<std::ptrdiff_t>(i));
                --i;
                continue;
            }
            ++i;
        }
        for (std::size_t i = 0; i + 1 < groups.size();)
        {
            if (Mergeable(segments, active, groups[i].first, groups[i + 1].last, tolerance))
            {
                groups[i].last = groups[i + 1].last;
                groups.erase(groups.begin() + static_cast<std::ptrdiff_t>(i + 1));
            }
            else { ++i; }
        }
        for (std::size_t output = 0; output < groups.size(); ++output)
        {
            const auto group = groups[output];
            const std::size_t first = active[group.first], last = active[group.last];
            if (group.first == group.last &&
                std::holds_alternative<SCArcSegment3d>(segments[first]))
            { result.simplifiedSegments.push_back(segments[first]); }
            else
            {
                result.simplifiedSegments.push_back(
                    SCLineSegment3d{SegmentStart(segments[first]), SegmentEnd(segments[last])});
            }
            CompensatedSum covered;
            for (std::size_t position = group.first; position <= group.last; ++position)
            {
                const std::size_t source = active[position];
                result.sourceMappings[source].simplifiedSegmentIndex = output;
                covered.Add(result.sourceSegmentLengths[source]);
            }
            const double geometryLength = std::holds_alternative<SCArcSegment3d>(result.simplifiedSegments.back())
                ? covered.Value() : Norm(SegmentEnd(result.simplifiedSegments.back()) -
                                         SegmentStart(result.simplifiedSegments.back()));
            if (!std::isfinite(geometryLength) || !std::isfinite(covered.Value()))
            { return fail(SCPathGeometryFailure::NonFiniteResult, first); }
            result.coverages.push_back({output, first, last,
                                        result.sourceCumulativeLengths[first],
                                        result.sourceCumulativeLengths[last + 1],
                                        covered.Value(), geometryLength});
        }
        for (std::size_t i = 1; i < result.simplifiedSegments.size(); ++i)
        {
            if (Norm(SegmentEnd(result.simplifiedSegments[i - 1]) -
                     SegmentStart(result.simplifiedSegments[i])) > tolerance.distanceTolerance)
            { return fail(SCPathGeometryFailure::DiscontinuousPath,
                          result.coverages[i - 1].lastSourceSegmentIndex,
                          result.coverages[i].firstSourceSegmentIndex); }
        }
        if (closure == SCPolylineClosure::Closed &&
            Norm(SegmentEnd(result.simplifiedSegments.back()) -
                 SegmentStart(result.simplifiedSegments.front())) > tolerance.distanceTolerance)
        { return fail(SCPathGeometryFailure::DiscontinuousPath,
                      result.coverages.back().lastSourceSegmentIndex,
                      result.coverages.front().firstSourceSegmentIndex); }
        const bool onlyLines = std::all_of(result.simplifiedSegments.begin(),
                                          result.simplifiedSegments.end(), [](const auto& segment)
                                          { return std::holds_alternative<SCLineSegment3d>(segment); });
        if (onlyLines && result.simplifiedSegments.size() <
            (closure == SCPolylineClosure::Closed ? 3u : 1u))
        { return fail(SCPathGeometryFailure::InsufficientRemainingPoints); }
        if (closure == SCPolylineClosure::Closed && result.simplifiedSegments.size() == 1)
        {
            const auto* arc = std::get_if<SCArcSegment3d>(&result.simplifiedSegments.front());
            if (!arc || std::abs(arc->signedSweepRad) != 2.0 * std::numbers::pi_v<double>)
            { return fail(SCPathGeometryFailure::InsufficientRemainingPoints); }
        }
        result.closure = closure;
        result.sourceTotalLength = total.Value();
        result.success = true;
        return result;
    }

    SCPolylineSimplificationResult SimplifyPolylineWithMapping(
        std::span<const SCPoint3d> points, const SCPathGeometryTolerance& tolerance,
        SCPolylineClosure closure)
    {
        SCPolylineSimplificationResult result;
        if (!tolerance.IsValid())
        { result.failure = SCPathGeometryFailure::InvalidTolerance; return result; }
        if (!FinitePoints(points, result.inputIndex))
        { result.failure = SCPathGeometryFailure::NonFiniteInput; return result; }
        if (!PathDetail::ValidClosure(closure))
        { result.failure = SCPathGeometryFailure::InvalidClosure; return result; }
        if (points.size() < 2)
        { result.failure = SCPathGeometryFailure::InsufficientPoints; return result; }
        const std::size_t count = closure == SCPolylineClosure::Closed ? points.size() : points.size() - 1;
        std::vector<SCPathSegment3d> segments;
        segments.reserve(count);
        for (std::size_t i = 0; i < count; ++i)
        {
            segments.emplace_back(SCLineSegment3d{points[i], points[(i + 1) % points.size()]});
        }
        result = SimplifyPolylineWithMapping(std::span<const SCPathSegment3d>(segments), tolerance, closure);
        result.inputKind = SCPolylineSimplificationInputKind::PointSequence;
        if (!result.success) { return result; }
        result.simplifiedPoints.push_back(points.front());
        for (std::size_t i = 0; i < result.simplifiedSegments.size(); ++i)
        {
            if (closure == SCPolylineClosure::Closed && i + 1 == result.simplifiedSegments.size())
            { break; }
            result.simplifiedPoints.push_back(SegmentEnd(result.simplifiedSegments[i]));
        }
        return result;
    }

    SCCornerGeometryResult2d EvaluateCornerGeometry(const ISCSegment2d& incoming,
                                                     const ISCSegment2d& outgoing,
                                                     const SCPathGeometryTolerance& tolerance)
    {
        SCCornerGeometryResult2d result;
        if (!tolerance.IsValid())
        { result.failure = SCPathGeometryFailure::InvalidTolerance; return result; }
        const std::array<const ISCSegment2d*, 2> segments{&incoming, &outgoing};
        for (std::size_t i = 0; i < segments.size(); ++i)
        {
            if (segments[i]->Kind() == SCSegmentKind2::Line)
            {
                const auto* line = dynamic_cast<const SCLineSegment2d*>(segments[i]);
                if (!line)
                { result.failure = SCPathGeometryFailure::UnsupportedSegment; result.inputIndex = i; return result; }
                if (!line->startPoint.IsValid() || !line->endPoint.IsValid())
                { result.failure = SCPathGeometryFailure::NonFiniteInput; result.inputIndex = i; return result; }
            }
            else if (segments[i]->Kind() == SCSegmentKind2::Arc)
            {
                const auto* arc = dynamic_cast<const SCArcSegment2d*>(segments[i]);
                if (!arc)
                { result.failure = SCPathGeometryFailure::UnsupportedSegment; result.inputIndex = i; return result; }
                if (!arc->center.IsValid() || !std::isfinite(arc->radius) ||
                    !std::isfinite(arc->startAngle) || !std::isfinite(arc->sweepAngle))
                { result.failure = SCPathGeometryFailure::NonFiniteInput; result.inputIndex = i; return result; }
            }
            else
            { result.failure = SCPathGeometryFailure::UnsupportedSegment; result.inputIndex = i; return result; }
        }
        for (std::size_t i = 0; i < segments.size(); ++i)
        {
            if (!segments[i]->StartPoint().IsValid() || !segments[i]->EndPoint().IsValid())
            { result.failure = SCPathGeometryFailure::NonFiniteResult; result.inputIndex = i; return result; }
        }
        std::array<SCVector2d, 2> tangents{};
        for (std::size_t i = 0; i < segments.size(); ++i)
        {
            const auto* segment = segments[i];
            if (segment->Kind() == SCSegmentKind2::Line)
            {
                const auto* line = dynamic_cast<const SCLineSegment2d*>(segment);
                if (!line) { result.failure = SCPathGeometryFailure::UnsupportedSegment; result.inputIndex = i; return result; }
                if (!line->startPoint.IsValid() || !line->endPoint.IsValid())
                { result.failure = SCPathGeometryFailure::NonFiniteInput; result.inputIndex = i; return result; }
                const SCVector2d delta = line->endPoint - line->startPoint;
                const double length = PathDetail::Norm(delta);
                if (!std::isfinite(length))
                { result.failure = SCPathGeometryFailure::NonFiniteResult; result.inputIndex = i; return result; }
                if (length <= tolerance.distanceTolerance)
                { result.failure = SCPathGeometryFailure::DegenerateDirection; result.inputIndex = i; return result; }
                if (!line->IsValid())
                { result.failure = SCPathGeometryFailure::Unrepresentable2d; result.inputIndex = i; return result; }
                tangents[i] = delta / length;
            }
            else if (segment->Kind() == SCSegmentKind2::Arc)
            {
                const auto* arc = dynamic_cast<const SCArcSegment2d*>(segment);
                if (!arc) { result.failure = SCPathGeometryFailure::UnsupportedSegment; result.inputIndex = i; return result; }
                if (!arc->center.IsValid() || !std::isfinite(arc->radius) ||
                    !std::isfinite(arc->startAngle) || !std::isfinite(arc->sweepAngle))
                { result.failure = SCPathGeometryFailure::NonFiniteInput; result.inputIndex = i; return result; }
                if (arc->radius <= tolerance.distanceTolerance ||
                    std::abs(arc->sweepAngle) <= tolerance.angleToleranceRad ||
                    std::abs(arc->sweepAngle) > 2.0 * std::numbers::pi_v<double>)
                { result.failure = SCPathGeometryFailure::DegenerateDirection; result.inputIndex = i; return result; }
                if (!arc->IsValid())
                { result.failure = SCPathGeometryFailure::Unrepresentable2d; result.inputIndex = i; return result; }
                const double angle = i == 0 ? arc->startAngle + arc->sweepAngle : arc->startAngle;
                const double sign = std::copysign(1.0, arc->sweepAngle);
                tangents[i] = SCVector2d{-sign * std::sin(angle), sign * std::cos(angle)};
            }
            else
            { result.failure = SCPathGeometryFailure::UnsupportedSegment; result.inputIndex = i; return result; }
        }
        if (PathDetail::Norm(incoming.EndPoint() - outgoing.StartPoint()) > tolerance.distanceTolerance)
        { result.failure = SCPathGeometryFailure::DiscontinuousPath; result.inputIndex = 0; result.relatedIndex = 1; return result; }
        const double cross = tangents[0].x * tangents[1].y - tangents[0].y * tangents[1].x;
        const double dot = tangents[0].x * tangents[1].x + tangents[0].y * tangents[1].y;
        const double turn = std::atan2(cross, dot);
        if (!std::isfinite(turn))
        { result.failure = SCPathGeometryFailure::NonFiniteResult; return result; }
        if (std::abs(turn) <= tolerance.angleToleranceRad)
        {
            result.interiorAngleRad = std::numbers::pi_v<double>;
            result.turn = SCCornerTurn2d::Straight;
        }
        else if (std::numbers::pi_v<double> - std::abs(turn) <= tolerance.angleToleranceRad)
        {
            result.signedTurnRad = std::numbers::pi_v<double>;
            result.turn = SCCornerTurn2d::Reversal;
        }
        else
        {
            result.signedTurnRad = turn;
            result.interiorAngleRad = std::numbers::pi_v<double> - std::abs(turn);
            result.turn = turn > 0.0 ? SCCornerTurn2d::Left : SCCornerTurn2d::Right;
        }
        result.success = true;
        return result;
    }
}
