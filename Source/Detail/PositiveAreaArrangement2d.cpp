#include "PolygonPositiveAreaIntersection2d.h"

#include "CurveArrangement2d.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "Core/Intersection.h"
#include "DirectedEdgeFans2d.h"
#include "ArrangementVertices2d.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "../Core/RingIntegral2d.h"
#include "SegmentKernel2d.h"

namespace Geometry::Detail
{
    namespace
    {
        struct SourceSegment
        {
            const ISCSegment2d* segment{nullptr};
            std::size_t ring{0};
            std::size_t index{0};
            double parameterTolerance{0.0};
            std::vector<double> parameters{0.0, 1.0};
        };

        struct ArrangementEvent
        {
            std::size_t source{0};
            SCPoint2d point{};
            double parameter{0.0};
        };

        struct HalfEdge
        {
            std::unique_ptr<ISCSegment2d> segment{};
            std::size_t from{0};
            std::size_t to{0};
            std::size_t twin{0};
            double angle{0.0};
            bool visited{false};
        };

        [[nodiscard]] SCPoint2d EvaluateSegment(const ISCSegment2d& segment, double parameter)
        {
            if (const auto* line = dynamic_cast<const SCLineSegment2d*>(&segment))
            {
                return {line->startPoint.x + (line->endPoint.x - line->startPoint.x) * parameter,
                        line->startPoint.y + (line->endPoint.y - line->startPoint.y) * parameter};
            }
            if (const auto* arc = dynamic_cast<const SCArcSegment2d*>(&segment))
            {
                const double angle = arc->startAngle + arc->sweepAngle * parameter;
                return {arc->center.x + arc->radius * std::cos(angle),
                        arc->center.y + arc->radius * std::sin(angle)};
            }
            return {};
        }

        [[nodiscard]] bool TangentAngle(const ISCSegment2d& segment, bool fromStart, double& angle)
        {
            SCVector2d tangent{};
            if (const auto* line = dynamic_cast<const SCLineSegment2d*>(&segment))
            {
                tangent = line->endPoint - line->startPoint;
            }
            else if (const auto* arc = dynamic_cast<const SCArcSegment2d*>(&segment))
            {
                const double angle = arc->startAngle + arc->sweepAngle * (fromStart ? 0.0 : 1.0);
                tangent = {-arc->radius * std::sin(angle) * arc->sweepAngle,
                           arc->radius * std::cos(angle) * arc->sweepAngle};
            }
            if (!std::isfinite(tangent.x) || !std::isfinite(tangent.y) ||
                (tangent.x == 0.0 && tangent.y == 0.0))
            {
                return false;
            }
            angle = std::atan2(tangent.y, tangent.x);
            return std::isfinite(angle);
        }

        [[nodiscard]] bool Near(double first, double second, double tolerance)
        {
            return std::abs(first - second) <= tolerance;
        }

        [[nodiscard]] bool SortOutgoingFansChecked(const std::vector<HalfEdge>& edges,
                                                    std::vector<std::vector<std::size_t>>& outgoing)
        {
            for (std::vector<std::size_t>& fan : outgoing)
            {
                for (const std::size_t edgeIndex : fan)
                {
                    if (edgeIndex >= edges.size() || !std::isfinite(edges[edgeIndex].angle))
                    {
                        return false;
                    }
                }
                // 同一点的连续圆弧端点会因三角函数舍入产生极小角差；在已验证
                // 切线有限的前提下，以拓扑次级键提供确定顺序，不把该正常情形
                // 误报为数值不确定。
                std::sort(fan.begin(), fan.end(), [&edges](const std::size_t lhs, const std::size_t rhs) {
                    if (edges[lhs].angle != edges[rhs].angle)
                    {
                        return edges[lhs].angle < edges[rhs].angle;
                    }
                    if (edges[lhs].to != edges[rhs].to)
                    {
                        return edges[lhs].to < edges[rhs].to;
                    }
                    return lhs < rhs;
                });
            }
            return true;
        }

        [[nodiscard]] bool IsFiniteKernelGeometry(const ISCSegment2d& segment)
        {
            if (const auto* line = dynamic_cast<const SCLineSegment2d*>(&segment))
            {
                const SCVector2d delta = line->endPoint - line->startPoint;
                return line->startPoint.IsValid() && line->endPoint.IsValid() &&
                       std::isfinite(delta.x) && std::isfinite(delta.y) &&
                       (delta.x != 0.0 || delta.y != 0.0);
            }
            if (const auto* arc = dynamic_cast<const SCArcSegment2d*>(&segment))
            {
                return arc->center.IsValid() && std::isfinite(arc->radius) && arc->radius > 0.0 &&
                       std::isfinite(arc->startAngle) && std::isfinite(arc->sweepAngle) &&
                       arc->sweepAngle != 0.0;
            }
            return false;
        }

        [[nodiscard]] bool AddParameter(SourceSegment& source, double parameter)
        {
            if (!std::isfinite(parameter) || parameter < 0.0 || parameter > 1.0)
            {
                return false;
            }
            for (double existing : source.parameters)
            {
                if (Near(existing, parameter, source.parameterTolerance))
                {
                    return true;
                }
            }
            source.parameters.push_back(parameter);
            return true;
        }

        [[nodiscard]] bool AddEvent(std::vector<ArrangementEvent>& events,
                                    std::vector<SourceSegment>& sources,
                                    std::size_t sourceIndex,
                                    const SCIntersectionPoint2d& point)
        {
            if (sourceIndex >= sources.size() || !point.IsValid() || !std::isfinite(point.parameterOnFirst) ||
                !std::isfinite(point.parameterOnSecond) || point.parameterOnFirst < 0.0 ||
                point.parameterOnFirst > 1.0 || point.parameterOnSecond < 0.0 || point.parameterOnSecond > 1.0)
            {
                return false;
            }
            if (!AddParameter(sources[sourceIndex], point.parameterOnFirst))
            {
                return false;
            }
            events.push_back({sourceIndex, point.point, point.parameterOnFirst});
            return true;
        }

        [[nodiscard]] bool SortAndDeduplicateParameters(SourceSegment& source)
        {
            std::sort(source.parameters.begin(), source.parameters.end());
            std::vector<double> normalized;
            normalized.reserve(source.parameters.size());
            for (const double parameter : source.parameters)
            {
                if (!std::isfinite(parameter) || parameter < 0.0 || parameter > 1.0)
                {
                    return false;
                }
                if (normalized.empty() || !Near(normalized.back(), parameter, source.parameterTolerance))
                {
                    normalized.push_back(parameter);
                }
            }
            if (normalized.size() < 2 || normalized.front() != 0.0 || normalized.back() != 1.0)
            {
                return false;
            }
            source.parameters = std::move(normalized);
            return true;
        }

        [[nodiscard]] bool ValidateOutgoingFans(const std::vector<HalfEdge>& edges,
                                                const std::vector<std::vector<std::size_t>>& outgoing)
        {
            for (const auto& fan : outgoing)
            {
                for (const std::size_t edgeIndex : fan)
                {
                    if (edgeIndex >= edges.size() || !std::isfinite(edges[edgeIndex].angle))
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        [[nodiscard]] std::unique_ptr<ISCSegment2d> MakeCheckedPiece(const ISCSegment2d& source,
                                                                       double start,
                                                                       double end)
        {
            if (!std::isfinite(start) || !std::isfinite(end) || start < 0.0 || end > 1.0 || !(end > start))
            {
                return nullptr;
            }
            if (const auto* line = dynamic_cast<const SCLineSegment2d*>(&source))
            {
                const SCPoint2d first = EvaluateSegment(*line, start);
                const SCPoint2d last = EvaluateSegment(*line, end);
                if (!first.IsValid() || !last.IsValid())
                {
                    return nullptr;
                }
                return std::make_unique<SCLineSegment2d>(first, last);
            }
            if (const auto* arc = dynamic_cast<const SCArcSegment2d*>(&source))
            {
                const double pieceSweep = arc->sweepAngle * (end - start);
                if (!std::isfinite(pieceSweep) || pieceSweep == 0.0)
                {
                    return nullptr;
                }
                return std::make_unique<SCArcSegment2d>(arc->center,
                                                        arc->radius,
                                                        arc->startAngle + arc->sweepAngle * start,
                                                        pieceSweep);
            }
            return nullptr;
        }
    }  // namespace

    CurveArrangementResult2d BuildPositiveAreaCurveArrangement2d(
        const std::vector<std::vector<std::unique_ptr<ISCSegment2d>>>& rings,
        double tolerance)
    {
        CurveArrangementResult2d result;
        if (!std::isfinite(tolerance) || tolerance <= 0.0)
        {
            return result;
        }
        result.failure = CurveArrangementFailure2d::None;

        std::vector<SourceSegment> sources;
        std::vector<ArrangementEvent> events;
        for (std::size_t ringIndex = 0; ringIndex < rings.size(); ++ringIndex)
        {
            for (std::size_t segmentIndex = 0; segmentIndex < rings[ringIndex].size(); ++segmentIndex)
            {
                const auto& segment = rings[ringIndex][segmentIndex];
                if (segment == nullptr || !IsFiniteKernelGeometry(*segment) || !IsKernelSegment(*segment))
                {
                    result.failure = CurveArrangementFailure2d::InvalidInput;
                    return result;
                }
                const double length = segment->Length();
                if (!std::isfinite(length) || length <= 0.0)
                {
                    result.failure = CurveArrangementFailure2d::InvalidInput;
                    return result;
                }
                sources.push_back({segment.get(), ringIndex, segmentIndex,
                                   std::min(0.5, tolerance / length), {0.0, 1.0}});
            }
        }

        for (std::size_t first = 0; first < sources.size(); ++first)
        {
            for (std::size_t second = first + 1; second < sources.size(); ++second)
            {
                const SCCheckedIntersection2d checked = CheckedIntersectSegments(*sources[first].segment,
                                                                                   *sources[second].segment,
                                                                                   tolerance);
                if (checked.status == SCCheckedGeometryStatus2d::InvalidInput ||
                    checked.status == SCCheckedGeometryStatus2d::NumericalIndeterminate)
                {
                    result.failure = checked.status == SCCheckedGeometryStatus2d::InvalidInput
                                          ? CurveArrangementFailure2d::InvalidInput
                                          : CurveArrangementFailure2d::NumericalIndeterminate;
                    return result;
                }
                const SCSegmentIntersection2d& hit = checked.intersection;
                if (hit.kind == SCIntersectionKind2d::Overlap)
                {
                    result.hasOverlap = true;
                    if (hit.pointCount != 2)
                    {
                        continue;
                    }
                    for (std::size_t pointIndex = 0; pointIndex < hit.pointCount; ++pointIndex)
                    {
                        const auto& point = hit.points[pointIndex];
                        if (!AddEvent(events, sources, first,
                                      SCIntersectionPoint2d{point.point, point.parameterOnFirst, point.parameterOnSecond}) ||
                            !AddEvent(events, sources, second,
                                      SCIntersectionPoint2d{point.point, point.parameterOnSecond, point.parameterOnFirst}))
                        {
                            result.failure = CurveArrangementFailure2d::EventInsertionFailure;
                            return result;
                        }
                    }
                    continue;
                }
                if (!hit.HasIntersection())
                {
                    continue;
                }
                for (std::size_t pointIndex = 0; pointIndex < hit.pointCount; ++pointIndex)
                {
                    const auto& point = hit.points[pointIndex];
                    if (!AddEvent(events, sources, first,
                                  SCIntersectionPoint2d{point.point, point.parameterOnFirst, point.parameterOnSecond}) ||
                        !AddEvent(events, sources, second,
                                  SCIntersectionPoint2d{point.point, point.parameterOnSecond, point.parameterOnFirst}))
                    {
                        result.failure = CurveArrangementFailure2d::EventInsertionFailure;
                        return result;
                    }
                }
            }
        }

        for (std::size_t first = 0; first < events.size(); ++first)
        {
            for (std::size_t second = first + 1; second < events.size(); ++second)
            {
                if (events[first].source == events[second].source &&
                    events[first].point.AlmostEquals(events[second].point, tolerance) &&
                    !Near(events[first].parameter, events[second].parameter,
                          sources[events[first].source].parameterTolerance))
                {
                    const double parameterTolerance = sources[events[first].source].parameterTolerance;
                    const bool firstInterior = events[first].parameter > parameterTolerance &&
                                               events[first].parameter < 1.0 - parameterTolerance;
                    const bool secondInterior = events[second].parameter > parameterTolerance &&
                                                events[second].parameter < 1.0 - parameterTolerance;
                    result.hasAmbiguousEvent = result.hasAmbiguousEvent || (firstInterior && secondInterior);
                }
            }
        }

        for (SourceSegment& source : sources)
        {
            if (!SortAndDeduplicateParameters(source))
            {
                result.failure = CurveArrangementFailure2d::EventOrderingFailure;
                return result;
            }
            for (std::size_t i = 1; i < source.parameters.size(); ++i)
            {
                const double start = source.parameters[i - 1];
                const double end = source.parameters[i];
                std::unique_ptr<ISCSegment2d> piece = MakeCheckedPiece(*source.segment, start, end);
                if (piece == nullptr)
                {
                    result.failure = CurveArrangementFailure2d::PieceConstructionFailure;
                    return result;
                }
                result.pieces.push_back({std::move(piece), source.ring, source.index, start, end});
            }
        }
        std::vector<ArrangementPiece2d> uniquePieces;
        uniquePieces.reserve(result.pieces.size());
        for (ArrangementPiece2d& candidate : result.pieces)
        {
            bool duplicate = false;
            for (const ArrangementPiece2d& existing : uniquePieces)
            {
                const bool sameEndpoints =
                    candidate.segment->StartPoint().AlmostEquals(existing.segment->StartPoint(), tolerance) &&
                    candidate.segment->EndPoint().AlmostEquals(existing.segment->EndPoint(), tolerance);
                const bool reversedEndpoints =
                    candidate.segment->StartPoint().AlmostEquals(existing.segment->EndPoint(), tolerance) &&
                    candidate.segment->EndPoint().AlmostEquals(existing.segment->StartPoint(), tolerance);
                if (!sameEndpoints && !reversedEndpoints)
                {
                    continue;
                }
                if (!EvaluateSegment(*candidate.segment, 0.5).AlmostEquals(
                        EvaluateSegment(*existing.segment, 0.5), tolerance))
                {
                    continue;
                }
                const SCCheckedIntersection2d checked =
                    CheckedIntersectSegments(*candidate.segment, *existing.segment, tolerance);
                if (checked.status == SCCheckedGeometryStatus2d::InvalidInput ||
                    checked.status == SCCheckedGeometryStatus2d::NumericalIndeterminate)
                {
                    result.failure = checked.status == SCCheckedGeometryStatus2d::InvalidInput
                                          ? CurveArrangementFailure2d::InvalidInput
                                          : CurveArrangementFailure2d::NumericalIndeterminate;
                    return result;
                }
                if (checked.intersection.kind == SCIntersectionKind2d::Overlap)
                {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate)
            {
                uniquePieces.push_back(std::move(candidate));
            }
        }
        result.pieces = std::move(uniquePieces);
        if (result.pieces.empty())
        {
            result.failure = CurveArrangementFailure2d::InvalidInput;
            return result;
        }

        std::vector<SCPoint2d> vertices;
        std::vector<HalfEdge> edges;
        std::vector<std::vector<std::size_t>> outgoing;
        for (const ArrangementPiece2d& piece : result.pieces)
        {
            if (piece.segment == nullptr)
            {
                result.failure = CurveArrangementFailure2d::InvalidInput;
                return result;
            }
            const std::size_t from = FindOrAddVertex2d(vertices, piece.segment->StartPoint(), tolerance);
            const std::size_t to = FindOrAddVertex2d(vertices, piece.segment->EndPoint(), tolerance);
            if (from == to)
            {
                result.failure = CurveArrangementFailure2d::InvalidInput;
                return result;
            }
            outgoing.resize(vertices.size());
            const std::size_t forward = edges.size();
            double forwardAngle = 0.0;
            if (!TangentAngle(*piece.segment, true, forwardAngle))
            {
                result.failure = CurveArrangementFailure2d::NumericalIndeterminate;
                return result;
            }
            edges.push_back({piece.segment->Clone(), from, to, forward + 1, forwardAngle, false});
            std::unique_ptr<ISCSegment2d> reversed = ReverseKernelSegment(*piece.segment);
            if (reversed == nullptr || !reversed->StartPoint().IsValid() || !reversed->EndPoint().IsValid())
            {
                result.failure = CurveArrangementFailure2d::ReverseEdgeFailure;
                return result;
            }
            edges.push_back({std::move(reversed), to, from, forward, 0.0, false});
            if (!TangentAngle(*edges.back().segment, true, edges.back().angle))
            {
                result.failure = CurveArrangementFailure2d::NumericalIndeterminate;
                return result;
            }
            outgoing[from].push_back(forward);
            outgoing[to].push_back(forward + 1);
        }
        if (!SortOutgoingFansChecked(edges, outgoing) || !ValidateOutgoingFans(edges, outgoing))
        {
            result.failure = CurveArrangementFailure2d::EventOrderingFailure;
            return result;
        }
        for (std::size_t start = 0; start < edges.size(); ++start)
        {
            if (edges[start].visited)
            {
                continue;
            }
            std::vector<std::shared_ptr<ISCSegment2d>> ringSegments;
            std::size_t current = start;
            bool closed = false;
            for (std::size_t guard = 0; guard <= edges.size(); ++guard)
            {
                if (current >= edges.size() || edges[current].visited)
                {
                    closed = current == start;
                    break;
                }
                edges[current].visited = true;
                ringSegments.push_back(std::shared_ptr<ISCSegment2d>(edges[current].segment->Clone().release()));
                current = NextFaceEdge(edges, outgoing, current);
                if (current == kInvalidDirectedEdge2d)
                {
                    result.failure = CurveArrangementFailure2d::EventOrderingFailure;
                    return result;
                }
            }
            if (!closed || ringSegments.size() < 2)
            {
                result.failure = CurveArrangementFailure2d::FaceClosureFailure;
                return result;
            }
            if (!ringSegments.front()->StartPoint().AlmostEquals(ringSegments.back()->EndPoint(), tolerance))
            {
                result.failure = CurveArrangementFailure2d::FaceClosureFailure;
                return result;
            }
            SCPolyline2d ring(std::move(ringSegments), SCPolylineClosure::Closed);
            const double faceArea = Geometry::Detail::ComputeSignedArea(ring);
            if (ring.IsValid() && std::isfinite(faceArea) && std::abs(faceArea) > 0.0)
            {
                result.faceRings.push_back(std::move(ring));
            }
        }
        result.success = true;
        result.failure = CurveArrangementFailure2d::None;
        return result;
    }
}  // namespace Geometry::Detail
