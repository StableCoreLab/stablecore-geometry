#include "CurveArrangement2d.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "Core/Intersection.h"
#include "DirectedEdgeFans2d.h"
#include "ArrangementVertices2d.h"
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

        [[nodiscard]] double TangentAngle(const ISCSegment2d& segment, bool fromStart)
        {
            constexpr double delta = 1e-7;
            const SCPoint2d first = fromStart ? segment.PointAt(0.0) : segment.PointAt(1.0 - delta);
            const SCPoint2d second = fromStart ? segment.PointAt(delta) : segment.PointAt(1.0);
            const SCVector2d direction = fromStart ? second - first : second - first;
            return std::atan2(direction.y, direction.x);
        }

        [[nodiscard]] bool Near(double first, double second, double tolerance)
        {
            return std::abs(first - second) <= tolerance;
        }

        void AddParameter(SourceSegment& source, double parameter)
        {
            if (!std::isfinite(parameter))
            {
                return;
            }
            parameter = std::clamp(parameter, 0.0, 1.0);
            for (double existing : source.parameters)
            {
                if (Near(existing, parameter, source.parameterTolerance))
                {
                    return;
                }
            }
            source.parameters.push_back(parameter);
        }
    }  // namespace

    CurveArrangementResult2d BuildCurveArrangement2d(
        const std::vector<std::vector<std::unique_ptr<ISCSegment2d>>>& rings,
        double tolerance)
    {
        CurveArrangementResult2d result;
        if (!std::isfinite(tolerance) || tolerance <= 0.0)
        {
            return result;
        }

        std::vector<SourceSegment> sources;
        std::vector<ArrangementEvent> events;
        for (std::size_t ringIndex = 0; ringIndex < rings.size(); ++ringIndex)
        {
            for (std::size_t segmentIndex = 0; segmentIndex < rings[ringIndex].size(); ++segmentIndex)
            {
                const auto& segment = rings[ringIndex][segmentIndex];
                if (segment == nullptr || !segment->IsValid() || !IsKernelSegment(*segment))
                {
                    return result;
                }
                const double length = segment->Length();
                if (!std::isfinite(length) || length <= tolerance)
                {
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
                const SCSegmentIntersection2d hit = IntersectKernelSegments(*sources[first].segment,
                                                                               *sources[second].segment,
                                                                               tolerance);
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
                        events.push_back({first, point.point, point.parameterOnFirst});
                        events.push_back({second, point.point, point.parameterOnSecond});
                        AddParameter(sources[first], point.parameterOnFirst);
                        AddParameter(sources[second], point.parameterOnSecond);
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
                    events.push_back({first, point.point, point.parameterOnFirst});
                    events.push_back({second, point.point, point.parameterOnSecond});
                    AddParameter(sources[first], point.parameterOnFirst);
                    AddParameter(sources[second], point.parameterOnSecond);
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
            std::sort(source.parameters.begin(), source.parameters.end());
            const SegmentSplitResult2d split = SplitAtParameters(*source.segment, source.parameters, tolerance);
            if (!split.success)
            {
                return {};
            }
            for (const SegmentSplitPiece2d& piece : split.pieces)
            {
                result.pieces.push_back({piece.segment->Clone(), source.ring, source.index,
                                         piece.sourceStart, piece.sourceEnd});
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
                if (!candidate.segment->PointAt(0.5).AlmostEquals(existing.segment->PointAt(0.5), tolerance))
                {
                    continue;
                }
                const SCSegmentIntersection2d hit =
                    IntersectKernelSegments(*candidate.segment, *existing.segment, tolerance);
                if (hit.kind == SCIntersectionKind2d::Overlap)
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
            return result;
        }

        std::vector<SCPoint2d> vertices;
        std::vector<HalfEdge> edges;
        std::vector<std::vector<std::size_t>> outgoing;
        for (const ArrangementPiece2d& piece : result.pieces)
        {
            if (piece.segment == nullptr)
            {
                return {};
            }
            const std::size_t from = FindOrAddVertex2d(vertices, piece.segment->StartPoint(), tolerance);
            const std::size_t to = FindOrAddVertex2d(vertices, piece.segment->EndPoint(), tolerance);
            if (from == to)
            {
                return {};
            }
            outgoing.resize(vertices.size());
            const std::size_t forward = edges.size();
            edges.push_back({piece.segment->Clone(), from, to, forward + 1,
                             TangentAngle(*piece.segment, true), false});
            edges.push_back({ReverseKernelSegment(*piece.segment), to, from, forward,
                             TangentAngle(*edges.back().segment, true), false});
            outgoing[from].push_back(forward);
            outgoing[to].push_back(forward + 1);
        }
        SortOutgoingFans(edges, outgoing);
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
            }
            if (!closed || ringSegments.size() < 2)
            {
                return {};
            }
            SCPolyline2d ring(std::move(ringSegments), SCPolylineClosure::Closed);
            if (ring.IsValid() && std::abs(Geometry::Detail::ComputeSignedArea(ring)) > tolerance * tolerance)
            {
                result.faceRings.push_back(std::move(ring));
            }
        }
        result.success = true;
        return result;
    }
}  // namespace Geometry::Detail
