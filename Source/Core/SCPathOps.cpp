#include "Geometry2d/SCPathOps.h"
#include "Support/StableNumerics.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../Detail/ArrangementVertices2d.h"
#include "../Detail/DirectedEdgeFans2d.h"
#include "Detail/PolygonNesting2d.h"
#include "../Detail/RingVertices2d.h"
#include "Detail/SegmentSubdivision2d.h"
#include "../Detail/TolerancePolicy2d.h"
#include "../Detail/RingIntegral2d.h"
#include "Core/PolygonRelation2d.h"
#include "Core/Boolean.h"
#include "Core/Editing.h"
#include "Core/Intersection.h"
#include "Core/Projection.h"
#include "Core/Relation.h"
#include "Core/ShapeOps.h"
#include "Core/Validation.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Support/Epsilon.h"
#include "Support/Geometry2d/Predicate2.h"

namespace Geometry
{
    namespace
    {
        struct DirectedEdge : Detail::DirectedEdge2d
        {
            double length{0.0};
            bool synthetic{false};
            bool visited{false};
        };

        struct RawSegment
        {
            SCPoint2d start{};
            SCPoint2d end{};
            bool synthetic{false};
        };

        struct RingCandidate
        {
            SCPolyline2d ring{};
            double area{0.0};
            double perimeter{0.0};
            double syntheticPerimeter{0.0};
            std::size_t branchVertexCount{0};
            std::size_t syntheticBranchVertexCount{0};
            double score{0.0};
        };

        struct VertexGraph2d
        {
            std::vector<SCPoint2d> vertices;
            std::vector<std::size_t> degrees;
        };

        [[nodiscard]] std::vector<SCPoint2d> SimplifyRingVertices(std::vector<SCPoint2d> points, double eps);

        [[nodiscard]] SCPolyline2d ClipPolylineLength(const SCPolyline2d& polyline,
                                                       double startLength,
                                                       double endLength,
                                                       double eps,
                                                       bool explicitTolerance)
        {
            if (!(eps > 0.0) || !std::isfinite(eps) || !std::isfinite(startLength) ||
                !std::isfinite(endLength) ||
                !(explicitTolerance ? polyline.IsValid(eps) : polyline.IsValid()) ||
                endLength < startLength)
            {
                return {};
            }

            const double total = explicitTolerance ? polyline.Length(eps) : polyline.Length();
            if (!std::isfinite(total))
            {
                return {};
            }
            startLength = std::clamp(startLength, 0.0, total);
            endLength = std::clamp(endLength, 0.0, total);
            if (endLength - startLength < eps)
            {
                return {};
            }

            std::vector<std::shared_ptr<ISCSegment2d>> clippedSegments;
            double cursor = 0.0;
            for (std::size_t i = 0; i < polyline.SegmentCount(); ++i)
            {
                const std::unique_ptr<ISCSegment2d> segment = polyline.SegmentAt(i);
                const auto* arcSegment = explicitTolerance
                    ? dynamic_cast<const SCArcSegment2d*>(segment.get()) : nullptr;
                const double segLen = arcSegment != nullptr && arcSegment->IsValid(eps)
                    ? arcSegment->Length(eps) : segment->Length();
                if (!(segLen > 0.0) || !std::isfinite(segLen))
                {
                    return {};
                }
                const double segStart = cursor;
                const double segEnd = cursor + segLen;

                if (segEnd < startLength - eps)
                {
                    cursor = segEnd;
                    continue;
                }
                if (segStart > endLength + eps)
                {
                    break;
                }

                const double localStart = std::clamp(startLength - segStart, 0.0, segLen);
                const double localEnd = std::clamp(endLength - segStart, 0.0, segLen);
                if (!(localEnd - localStart > eps))
                {
                    cursor = segEnd;
                    continue;
                }
                const double startRatio = localStart / segLen;
                const double endRatio = localEnd / segLen;
                if (segment->Kind() == SCSegmentKind2d::Line)
                {
                    const auto* line = dynamic_cast<const SCLineSegment2d*>(segment.get());
                    if (line == nullptr)
                    {
                        return {};
                    }
                    const auto pointAtRatio = [&](double ratio) {
                        return SCPoint2d{std::lerp(line->startPoint.x, line->endPoint.x, ratio),
                                         std::lerp(line->startPoint.y, line->endPoint.y, ratio)};
                    };
                    clippedSegments.push_back(std::make_shared<SCLineSegment2d>(
                        pointAtRatio(startRatio), pointAtRatio(endRatio)));
                }
                else if (segment->Kind() == SCSegmentKind2d::Arc)
                {
                    const auto* arc = dynamic_cast<const SCArcSegment2d*>(segment.get());
                    if (arc == nullptr)
                    {
                        return {};
                    }
                    auto clippedArc = std::make_shared<SCArcSegment2d>(
                        arc->center, arc->radius, arc->startAngle + arc->sweepAngle * startRatio,
                        arc->sweepAngle * (endRatio - startRatio));
                    if (!(clippedArc->IsValid() ||
                          (explicitTolerance && clippedArc->IsValid(eps))))
                    {
                        return {};
                    }
                    clippedSegments.push_back(std::move(clippedArc));
                }
                else
                {
                    return {};
                }

                cursor = segEnd;
            }

            SCPolyline2d clipped(std::move(clippedSegments), SCPolylineClosure::Open);
            return (explicitTolerance ? clipped.IsValid(eps) : clipped.IsValid())
                ? clipped : SCPolyline2d{};
        }

        [[nodiscard]] SCPolygon2d BuildHalfPlaneClipPolygon(const SCPolygon2d& polygon,
                                                          const SCLineSegment2d& cutter,
                                                          bool keepLeft,
                                                          double eps)
        {
            if (!polygon.IsValid(eps))
            {
                return {};
            }

            const SCBox2d bounds = polygon.Bounds(eps);
            if (!bounds.IsValid())
            {
                return {};
            }

            const SCVector2d direction = cutter.endPoint - cutter.startPoint;
            const double length = std::hypot(direction.x, direction.y);
            if (!std::isfinite(length) || length <= eps)
            {
                return {};
            }

            const SCVector2d tangent = direction / length;
            SCVector2d normal{-tangent.y, tangent.x};
            if (!keepLeft)
            {
                normal = normal * -1.0;
            }

            const SCPoint2d minPoint = bounds.MinPoint();
            const SCPoint2d maxPoint = bounds.MaxPoint();
            const double diagonal = std::hypot(maxPoint.x - minPoint.x,
                                               maxPoint.y - minPoint.y);
            const double extent = std::max(1.0, 8.0 * diagonal + 1.0);

            const SCVector2d along = tangent * extent;
            const SCVector2d side = normal * (2.0 * extent);
            const SCPoint2d center{std::midpoint(minPoint.x, maxPoint.x),
                                   std::midpoint(minPoint.y, maxPoint.y)};
            const SCVector2d lineNormal{-tangent.y, tangent.x};
            const SCPoint2d anchor = center - lineNormal * Dot(center - cutter.startPoint, lineNormal);
            const SCPoint2d lineStart = anchor - along;
            const SCPoint2d lineEnd = anchor + along;

            SCPolyline2d ring(
                {
                    lineStart,
                    lineEnd,
                    lineEnd + side,
                    lineStart + side,
                },
                SCPolylineClosure::Closed);
            if (Orientation(ring) != SCRingOrientation2d::CounterClockwise)
            {
                ring = Reverse(ring);
            }

            const SCPolygon2d clip(ring);
            if (!clip.IsValid(eps) || !Validate(clip, eps).valid)
            {
                return {};
            }
            return clip;
        }

        void CollectRawSegments(const SCPolyline2d& polyline, std::vector<RawSegment>& segments, double eps)
        {
            const SCPolyline2d normalized = Normalize(polyline, eps);
            if (!normalized.IsValid(eps) || normalized.PointCount() < 2)
            {
                return;
            }

            const std::size_t segmentCount =
                normalized.IsClosed() ? normalized.PointCount() : normalized.PointCount() - 1;
            for (std::size_t i = 0; i < segmentCount; ++i)
            {
                const SCPoint2d start = normalized.VertexAt(i);
                const SCPoint2d end = normalized.VertexAt((i + 1) % normalized.PointCount());
                if (!start.AlmostEquals(end, eps))
                {
                    segments.push_back(RawSegment{start, end, false});
                }
            }
        }

        void AppendPolygonBoundaries(const SCPolygon2d& polygon, SCMultiPolyline2d& boundaries, double eps)
        {
            if (!polygon.OuterRing().IsValid(eps))
            {
                return;
            }

            boundaries.Add(polygon.OuterRing());
            for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
            {
                boundaries.Add(polygon.HoleAt(i));
            }
        }

        [[nodiscard]] SCPolygon2d SelectLargestValidPolygon(const SCMultiPolygon2d& polygons, double eps)
        {
            SCPolygon2d best;
            double bestArea = 0.0;
            for (std::size_t i = 0; i < polygons.Size(); ++i)
            {
                if (!polygons[i].IsValid(eps))
                {
                    continue;
                }

                const double candidateArea = std::abs(polygons[i].Area(eps));
                if (candidateArea > bestArea)
                {
                    best = polygons[i];
                    bestArea = candidateArea;
                }
            }
            return best;
        }

        [[nodiscard]] SCPolygon2d RebuildLargestPolygonWithCandidates(const SCMultiPolyline2d& boundaries,
                                                                    const std::vector<double>& epsCandidates)
        {
            for (double candidateEps : epsCandidates)
            {
                const SCMultiPolygon2d rebuilt = BuildMultiPolygonByLines(boundaries, candidateEps);
                if (rebuilt.Size() == 1 && rebuilt[0].IsValid(candidateEps))
                {
                    return rebuilt[0];
                }

                SCPolygon2d best = SelectLargestValidPolygon(rebuilt, candidateEps);
                if (best.IsValid(candidateEps))
                {
                    return best;
                }
            }

            return {};
        }

        [[nodiscard]] SCMultiPolyline2d SimplifyBoundaryPolylines(const SCMultiPolyline2d& boundaries, double eps)
        {
            SCMultiPolyline2d simplifiedBoundaries;
            for (std::size_t boundaryIndex = 0; boundaryIndex < boundaries.Size(); ++boundaryIndex)
            {
                const SCPolyline2d boundary = boundaries[boundaryIndex];
                std::vector<SCPoint2d> points;
                points.reserve(boundary.PointCount());
                for (std::size_t pointIndex = 0; pointIndex < boundary.PointCount(); ++pointIndex)
                {
                    points.push_back(boundary.VertexAt(pointIndex));
                }

                std::vector<SCPoint2d> simplifiedPoints = SimplifyRingVertices(std::move(points), eps);
                const bool isClosed = boundary.IsClosed();
                const std::size_t minimumPointCount = isClosed ? 3U : 2U;
                if (simplifiedPoints.size() < minimumPointCount)
                {
                    continue;
                }

                SCPolyline2d simplifiedBoundary(std::move(simplifiedPoints),
                                              isClosed ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
                if (simplifiedBoundary.IsValid(eps))
                {
                    simplifiedBoundaries.Add(std::move(simplifiedBoundary));
                }
            }

            return simplifiedBoundaries;
        }

        [[nodiscard]] std::vector<RawSegment> CollectRawSegments(const SCMultiPolyline2d& polylines, double eps)
        {
            std::vector<RawSegment> segments;
            for (std::size_t i = 0; i < polylines.Size(); ++i)
            {
                CollectRawSegments(polylines[i], segments, eps);
            }
            return segments;
        }

        [[nodiscard]] std::vector<RawSegment> SubdivideRawSegments(const std::vector<RawSegment>& rawSegments,
                                                                   double eps)
        {
            std::vector<SCLineSegment2d> segments;
            segments.reserve(rawSegments.size());
            for (const RawSegment& rawSegment : rawSegments)
            {
                segments.emplace_back(rawSegment.start, rawSegment.end);
            }

            const std::vector<Detail::SubdividedLineSegment2d> subdivisions = Detail::SubdivideLineSegments(
                segments,
                eps,
                [eps](const SCLineSegment2d& segment) {
                    return Detail::ComputeSubdivisionParameterTolerance2d(segment.Length(), eps);
                });

            std::vector<RawSegment> splitSegments;
            splitSegments.reserve(subdivisions.size());
            for (const Detail::SubdividedLineSegment2d& subdivision : subdivisions)
            {
                splitSegments.push_back(
                    RawSegment{subdivision.start, subdivision.end, rawSegments[subdivision.sourceIndex].synthetic});
            }

            return splitSegments;
        }

        [[nodiscard]] VertexGraph2d BuildVertexGraph(const std::vector<RawSegment>& segments, double eps)
        {
            VertexGraph2d graph;
            for (const RawSegment& segment : segments)
            {
                if (segment.start.AlmostEquals(segment.end, eps))
                {
                    continue;
                }

                const std::size_t from = Detail::FindOrAddVertex2d(graph.vertices, segment.start, eps);
                const std::size_t to = Detail::FindOrAddVertex2d(graph.vertices, segment.end, eps);
                if (graph.degrees.size() < graph.vertices.size())
                {
                    graph.degrees.resize(graph.vertices.size(), 0);
                }
                if (from != to)
                {
                    ++graph.degrees[from];
                    ++graph.degrees[to];
                }
            }
            return graph;
        }

        [[nodiscard]] SCPolygonBuildDiagnostics2d AnalyzeBuildInput(const SCMultiPolyline2d& lines,
                                                                     double eps)
        {
            SCPolygonBuildDiagnostics2d diagnostics;
            diagnostics.inputPolylineCount = lines.Size();
            std::vector<RawSegment> segments;
            for (std::size_t lineIndex = 0; lineIndex < lines.Size(); ++lineIndex)
            {
                const SCPolyline2d& line = lines[lineIndex];
                if (!line.IsValid(eps) || line.PointCount() < 2) continue;
                const std::size_t count = line.IsClosed() ? line.PointCount() : line.PointCount() - 1;
                for (std::size_t i = 0; i < count; ++i)
                {
                    const SCPoint2d start = line.VertexAt(i);
                    const SCPoint2d end = line.VertexAt((i + 1) % line.PointCount());
                    if (!start.AlmostEquals(end, eps)) segments.push_back({start, end, false});
                }
            }
            diagnostics.inputSegmentCount = segments.size();
            const VertexGraph2d graph = BuildVertexGraph(segments, eps);
            diagnostics.uniqueVertexCount = graph.vertices.size();
            for (const std::size_t degree : graph.degrees)
            {
                if (degree == 1U)
                {
                    ++diagnostics.danglingEndpointCount;
                }
                else if (degree > 2U)
                {
                    ++diagnostics.branchVertexCount;
                }
            }
            diagnostics.inferredSyntheticEdgeCount = diagnostics.danglingEndpointCount / 2U;
            return diagnostics;
        }

        [[nodiscard]] std::vector<RawSegment> PruneBranchLeaves(std::vector<RawSegment> segments,
                                                                double eps)
        {
            bool changed = true;
            while (changed)
            {
                changed = false;
                const VertexGraph2d graph = BuildVertexGraph(segments, eps);
                std::vector<RawSegment> kept;
                kept.reserve(segments.size());
                for (const RawSegment& segment : segments)
                {
                    const auto degreeAt = [&](const SCPoint2d& point) {
                        for (std::size_t i = 0; i < graph.vertices.size(); ++i)
                        {
                            if (graph.vertices[i].AlmostEquals(point, eps)) return graph.degrees[i];
                        }
                        return std::size_t{0};
                    };
                    const std::size_t fromDegree = degreeAt(segment.start);
                    const std::size_t toDegree = degreeAt(segment.end);
                    if ((fromDegree == 1U && toDegree > 2U) ||
                        (toDegree == 1U && fromDegree > 2U))
                    {
                        changed = true;
                        continue;
                    }
                    kept.push_back(segment);
                }
                segments = std::move(kept);
            }
            return segments;
        }

        [[nodiscard]] SCMultiPolyline2d SegmentPolylines(const std::vector<RawSegment>& segments)
        {
            SCMultiPolyline2d lines;
            for (const RawSegment& segment : segments)
            {
                lines.Add(SCPolyline2d({segment.start, segment.end}, SCPolylineClosure::Open));
            }
            return lines;
        }

        [[nodiscard]] std::vector<RawSegment> RemoveDuplicateSegments(const std::vector<RawSegment>& segments,
                                                                      double eps)
        {
            std::vector<RawSegment> unique;
            std::vector<SCPoint2d> vertices;
            std::unordered_map<std::uint64_t, std::size_t> edgeKeys;
            unique.reserve(segments.size());
            for (const RawSegment& segment : segments)
            {
                if (segment.start.AlmostEquals(segment.end, eps))
                {
                    continue;
                }

                const std::size_t from = Detail::FindOrAddVertex2d(vertices, segment.start, eps);
                const std::size_t to = Detail::FindOrAddVertex2d(vertices, segment.end, eps);
                if (from == to)
                {
                    continue;
                }

                const std::uint64_t key = Detail::MakeUndirectedEdgeKey2d(from, to);
                const auto [it, inserted] = edgeKeys.emplace(key, unique.size());
                if (inserted)
                {
                    unique.push_back(segment);
                } else if (unique[it->second].synthetic && !segment.synthetic)
                {
                    unique[it->second] = segment;
                }
            }
            return unique;
        }

        [[nodiscard]] double ComputeRepairTolerance(const std::vector<RawSegment>& segments, double eps)
        {
            double totalLength = 0.0;
            double maxLength = 0.0;
            std::size_t counted = 0;
            for (const RawSegment& segment : segments)
            {
                const double length = (segment.end - segment.start).Length();
                if (length <= eps)
                {
                    continue;
                }
                totalLength += length;
                maxLength = std::max(maxLength, length);
                ++counted;
            }

            if (counted == 0)
            {
                return 16.0 * eps;
            }

            const double averageLength = totalLength / static_cast<double>(counted);
            return std::max(16.0 * eps, std::min(0.2 * averageLength, 0.25 * maxLength));
        }

        void AutoCloseDanglingEndpoints(std::vector<RawSegment>& segments, double repairTol, double eps)
        {
            const VertexGraph2d graph = BuildVertexGraph(segments, eps);
            std::vector<std::size_t> endpoints;
            for (std::size_t i = 0; i < graph.vertices.size(); ++i)
            {
                if (graph.degrees[i] == 1)
                {
                    endpoints.push_back(i);
                }
            }

            std::vector<bool> used(endpoints.size(), false);
            for (std::size_t i = 0; i < endpoints.size(); ++i)
            {
                if (used[i])
                {
                    continue;
                }

                const SCPoint2d& first = graph.vertices[endpoints[i]];
                std::size_t bestIndex = static_cast<std::size_t>(-1);
                double bestDistance = repairTol;
                for (std::size_t j = i + 1; j < endpoints.size(); ++j)
                {
                    if (used[j])
                    {
                        continue;
                    }

                    const double distance = (graph.vertices[endpoints[j]] - first).Length();
                    if (distance <= bestDistance && distance > eps)
                    {
                        bestDistance = distance;
                        bestIndex = j;
                    }
                }

                if (bestIndex != static_cast<std::size_t>(-1))
                {
                    used[i] = true;
                    used[bestIndex] = true;
                    segments.push_back(RawSegment{first, graph.vertices[endpoints[bestIndex]], true});
                }
            }
        }

        void AutoExtendDanglingEndpoints(std::vector<RawSegment>& segments, double repairTol, double eps)
        {
            const VertexGraph2d graph = BuildVertexGraph(segments, eps);
            const std::size_t originalCount = segments.size();
            for (std::size_t vertexIndex = 0; vertexIndex < graph.vertices.size(); ++vertexIndex)
            {
                if (graph.degrees[vertexIndex] != 1)
                {
                    continue;
                }

                const SCPoint2d& endpoint = graph.vertices[vertexIndex];
                SCSegmentProjection2d bestProjection;
                bool found = false;
                for (std::size_t segmentIndex = 0; segmentIndex < originalCount; ++segmentIndex)
                {
                    const RawSegment& segment = segments[segmentIndex];
                    if (endpoint.AlmostEquals(segment.start, eps) || endpoint.AlmostEquals(segment.end, eps))
                    {
                        continue;
                    }

                    const SCSegmentProjection2d projection =
                        ProjectPointToLineSegment(endpoint, SCLineSegment2d(segment.start, segment.end), true);
                    if (!projection.IsValid() || !projection.isOnSegment)
                    {
                        continue;
                    }
                    if (projection.parameter <= eps || projection.parameter >= 1.0 - eps)
                    {
                        continue;
                    }
                    const double gap=std::hypot(endpoint.x-projection.point.x,endpoint.y-projection.point.y);
                    if (!std::isfinite(gap) || gap > repairTol || gap <= eps)
                    {
                        continue;
                    }

                    if (!found || projection.distanceSquared < bestProjection.distanceSquared)
                    {
                        bestProjection = projection;
                        found = true;
                    }
                }

                if (found)
                {
                    segments.push_back(RawSegment{endpoint, bestProjection.point, true});
                }
            }
        }

        [[nodiscard]] std::vector<RawSegment> PruneDanglingSegments(std::vector<RawSegment> segments, double eps)
        {
            bool changed = true;
            while (changed)
            {
                changed = false;
                const VertexGraph2d graph = BuildVertexGraph(segments, eps);
                std::vector<RawSegment> kept;
                kept.reserve(segments.size());
                for (const RawSegment& segment : segments)
                {
                    const std::size_t from = Detail::FindVertexIndex2d(graph.vertices, segment.start, eps);
                    const std::size_t to = Detail::FindVertexIndex2d(graph.vertices, segment.end, eps);
                    if (from == Detail::kInvalidVertexIndex2d || to == Detail::kInvalidVertexIndex2d ||
                        from >= graph.degrees.size() || to >= graph.degrees.size())
                    {
                        continue;
                    }
                    if (graph.degrees[from] <= 1 || graph.degrees[to] <= 1)
                    {
                        changed = true;
                        continue;
                    }
                    kept.push_back(segment);
                }
                segments = std::move(kept);
            }
            return segments;
        }

        void AppendSplitEdges(const std::vector<RawSegment>& segments,
                              std::vector<SCPoint2d>& vertices,
                              std::vector<DirectedEdge>& edges,
                              std::vector<std::vector<std::size_t>>& outgoing,
                              std::unordered_set<std::uint64_t>& edgeKeys,
                              double eps)
        {
            for (const RawSegment& segment : segments)
            {
                if (segment.start.AlmostEquals(segment.end, eps))
                {
                    continue;
                }

                const std::size_t from = Detail::FindOrAddVertex2d(vertices, segment.start, eps);
                const std::size_t to = Detail::FindOrAddVertex2d(vertices, segment.end, eps);
                if (from == to)
                {
                    continue;
                }

                const std::uint64_t key = Detail::MakeUndirectedEdgeKey2d(from, to);
                if (!edgeKeys.insert(key).second)
                {
                    continue;
                }

                const SCVector2d forward = vertices[to] - vertices[from];
                const SCVector2d backward = vertices[from] - vertices[to];

                const std::size_t forwardIndex = edges.size();
                edges.push_back(DirectedEdge{{from, to, forwardIndex + 1, std::atan2(forward.y, forward.x)},
                                             forward.Length(),
                                             segment.synthetic,
                                             false});
                edges.push_back(DirectedEdge{{to, from, forwardIndex, std::atan2(backward.y, backward.x)},
                                             backward.Length(),
                                             segment.synthetic,
                                             false});

                const std::size_t required = std::max(from, to) + 1;
                if (outgoing.size() < required)
                {
                    outgoing.resize(required);
                }
                outgoing[from].push_back(forwardIndex);
                outgoing[to].push_back(forwardIndex + 1);
            }
        }

        [[nodiscard]] std::vector<SCPoint2d> SimplifyRingVertices(std::vector<SCPoint2d> points, double eps)
        {
            return Detail::NormalizeRingVertices2d(std::move(points), eps);
        }

        [[nodiscard]] bool RejectRingCandidate(const RingCandidate& candidate,
                                               double repairTol,
                                               double eps)
        {
            const SCBox2d bounds = candidate.ring.Bounds(eps);
            const double diagonal = std::hypot(bounds.Width(), bounds.Height());
            const auto absolute = Detail::StableProduct(eps, eps, 256);
            const auto relative = candidate.syntheticPerimeter > 0.0
                ? Detail::StableProduct(diagonal, std::max(repairTol, 16 * eps), 1e-3)
                : Detail::StableProduct(diagonal, eps, 1.0);
            if (!absolute || !relative) return true;
            const double areaTol = std::max(*absolute, *relative);
            if (candidate.area <= areaTol || candidate.perimeter <= eps)
            {
                return true;
            }

            if (candidate.syntheticPerimeter <= eps)
            {
                return false;
            }

            const double syntheticRatio = candidate.syntheticPerimeter / candidate.perimeter;
            const bool syntheticDominated =
                syntheticRatio >= 0.25 &&
                candidate.area <= candidate.syntheticPerimeter * std::max(repairTol, 16.0 * eps);
            const bool syntheticBranchDominated =
                candidate.syntheticBranchVertexCount > 0 &&
                candidate.syntheticBranchVertexCount * 2U >= candidate.branchVertexCount + 1U &&
                syntheticRatio >= 0.2 &&
                candidate.area <= candidate.syntheticPerimeter * std::max(1.5 * repairTol, 24.0 * eps);
            return syntheticDominated || syntheticBranchDominated;
        }

        [[nodiscard]] double ComputeRingCandidateScore(double area,
                                                       double syntheticPerimeter,
                                                       std::size_t branchVertexCount,
                                                       std::size_t syntheticBranchVertexCount,
                                                       double repairTol,
                                                       double eps)
        {
            const double scale = std::max(repairTol, 16.0 * eps);
            const double syntheticPenalty = syntheticPerimeter * scale;
            const double branchPenalty = static_cast<double>(branchVertexCount) * 0.5 * scale;
            const double syntheticBranchPenalty = static_cast<double>(syntheticBranchVertexCount) * 1.5 * scale;
            return area - syntheticPenalty - branchPenalty - syntheticBranchPenalty;
        }

        [[nodiscard]] std::vector<RingCandidate> ExtractCandidateRings(
            std::vector<DirectedEdge>& edges,
            const std::vector<SCPoint2d>& vertices,
            const std::vector<std::vector<std::size_t>>& outgoing,
            double repairTol,
            double eps)
        {
            std::vector<RingCandidate> rings;
            for (std::size_t start = 0; start < edges.size(); ++start)
            {
                if (edges[start].visited)
                {
                    continue;
                }

                std::vector<SCPoint2d> loopPoints;
                double perimeter = 0.0;
                double syntheticPerimeter = 0.0;
                std::size_t branchVertexCount = 0;
                std::size_t syntheticBranchVertexCount = 0;
                std::size_t current = start;
                bool closed = false;
                for (std::size_t steps = 0; steps <= edges.size(); ++steps)
                {
                    DirectedEdge& edge = edges[current];
                    if (edge.visited)
                    {
                        break;
                    }

                    edge.visited = true;
                    loopPoints.push_back(vertices[edge.from]);
                    perimeter += edge.length;
                    if (edge.synthetic)
                    {
                        syntheticPerimeter += edge.length;
                    }
                    if (edge.from < outgoing.size() && outgoing[edge.from].size() > 2U)
                    {
                        ++branchVertexCount;
                        if (edge.synthetic)
                        {
                            ++syntheticBranchVertexCount;
                        }
                    }

                    const std::size_t next = Detail::NextFaceEdge(edges, outgoing, current);
                    if (next == Detail::kInvalidDirectedEdge2d)
                    {
                        break;
                    }
                    if (next == start)
                    {
                        closed = true;
                        break;
                    }
                    current = next;
                }

                if (!closed)
                {
                    continue;
                }

                std::vector<SCPoint2d> simplified = SimplifyRingVertices(std::move(loopPoints), eps);
                if (simplified.size() < 3)
                {
                    continue;
                }

                SCPolyline2d ring(std::move(simplified), SCPolylineClosure::Closed);
                if (!Validate(ring, eps).valid)
                {
                    continue;
                }
                if (Detail::ComputeSignedArea(ring, eps) <= 0.0)
                {
                    continue;
                }

                const double area = std::abs(SCPolygon2d(ring).Area(eps));
                const double score = ComputeRingCandidateScore(
                    area, syntheticPerimeter, branchVertexCount, syntheticBranchVertexCount, repairTol, eps);
                if (!std::isfinite(area) || !std::isfinite(perimeter) || !std::isfinite(score))
                {
                    continue;
                }
                const RingCandidate candidate{
                    ring, area, perimeter, syntheticPerimeter, branchVertexCount, syntheticBranchVertexCount, score};
                if (RejectRingCandidate(candidate, repairTol, eps))
                {
                    continue;
                }

                rings.push_back(candidate);
            }

            std::stable_sort(rings.begin(), rings.end(), [](const RingCandidate& lhs, const RingCandidate& rhs) {
                if (lhs.score != rhs.score)
                {
                    return lhs.score > rhs.score;
                }
                if (lhs.area != rhs.area)
                {
                    return lhs.area > rhs.area;
                }
                return lhs.perimeter < rhs.perimeter;
            });

            return rings;
        }

        [[nodiscard]] std::size_t ComputeDepth(const std::vector<std::size_t>& parents, std::size_t index)
        {
            std::size_t depth = 0;
            while (parents[index] != static_cast<std::size_t>(-1))
            {
                ++depth;
                index = parents[index];
            }
            return depth;
        }

        [[nodiscard]] SCMultiPolygon2d BuildFilledPolygonsFromCandidateRings(const std::vector<RingCandidate>& rings,
                                                                           double eps)
        {
            SCMultiPolygon2d result;
            if (rings.empty())
            {
                return result;
            }

            std::vector<SCPolygon2d> loopPolygons;
            loopPolygons.reserve(rings.size());
            for (const RingCandidate& candidate : rings)
            {
                SCPolygon2d polygon(candidate.ring);
                if (polygon.IsValid(eps))
                {
                    loopPolygons.push_back(std::move(polygon));
                }
            }
            if (loopPolygons.empty())
            {
                return result;
            }

            const std::vector<std::size_t> parents = Detail::BuildPolygonParents2d(loopPolygons, eps);
            std::vector<std::size_t> depths(loopPolygons.size(), 0);
            for (std::size_t i = 0; i < loopPolygons.size(); ++i)
            {
                depths[i] = ComputeDepth(parents, i);
            }

            for (std::size_t i = 0; i < loopPolygons.size(); ++i)
            {
                if ((depths[i] % 2U) != 0U)
                {
                    continue;
                }

                SCPolyline2d outerRing = loopPolygons[i].OuterRing();
                if (Detail::ComputeSignedArea(outerRing, eps) < 0.0)
                {
                    outerRing = Reverse(outerRing);
                }

                std::vector<SCPolyline2d> holes;
                for (std::size_t j = 0; j < loopPolygons.size(); ++j)
                {
                    if (parents[j] != i || (depths[j] % 2U) == 0U)
                    {
                        continue;
                    }

                    SCPolyline2d holeRing = loopPolygons[j].OuterRing();
                    if (Detail::ComputeSignedArea(holeRing, eps) > 0.0)
                    {
                        holeRing = Reverse(holeRing);
                    }
                    holes.push_back(std::move(holeRing));
                }

                SCPolygon2d polygon(outerRing, std::move(holes));
                if (polygon.IsValid(eps) && Validate(polygon, eps).valid)
                {
                    result.Add(std::move(polygon));
                }
            }

            return result;
        }
    }  // namespace

    SCPolyline2d SubPolyline(const SCPolyline2d& polyline, double startLength, double endLength)
    {
        return ClipPolylineLength(polyline, startLength, endLength,
                                  Geometry::kDefaultEpsilon, false);
    }

    SCPolyline2d SubPolyline(const SCPolyline2d& polyline,
                             double startLength,
                             double endLength,
                             double epsilon)
    {
        return ClipPolylineLength(polyline, startLength, endLength, epsilon, true);
    }

    namespace
    {
        [[nodiscard]] SCPolygonCutResult2d ComputePolygonCut(const SCPolygon2d& polygon,
                                                              const SCLineSegment2d& cutter,
                                                              double eps)
        {
            SCPolygonCutResult2d result;
            const double cutterLength = std::hypot(cutter.endPoint.x - cutter.startPoint.x,
                                                   cutter.endPoint.y - cutter.startPoint.y);
            if (!polygon.IsValid(eps) || !(eps > 0.0) || !std::isfinite(eps) ||
                !cutter.startPoint.IsValid() || !cutter.endPoint.IsValid() ||
                !std::isfinite(cutterLength) || !(cutterLength > eps))
            {
                return result;
            }
            const SCPolygon2d leftClip = BuildHalfPlaneClipPolygon(polygon, cutter, true, eps);
            const SCPolygon2d rightClip = BuildHalfPlaneClipPolygon(polygon, cutter, false, eps);
            if (!leftClip.IsValid(eps) || !rightClip.IsValid(eps))
            {
                return result;
            }
            result.left = Intersect(polygon, leftClip, eps);
            result.right = Intersect(polygon, rightClip, eps);
            result.success = !result.left.IsEmpty() && !result.right.IsEmpty();
            if (!result.success)
            {
                result.left = {};
                result.right = {};
            }
            return result;
        }
    }  // namespace

    std::optional<SCPolygonCutParts2d> CutPolygonParts(const SCPolygon2d& polygon,
                                                       const SCLineSegment2d& cutter,
                                                       double eps)
    {
        SCPolygonCutResult2d result = ComputePolygonCut(polygon, cutter, eps);
        if (!result.success)
        {
            return std::nullopt;
        }
        return SCPolygonCutParts2d{std::move(result.left), std::move(result.right)};
    }

    SCPolygonCutResult2d CutPolygon(const SCPolygon2d& polygon, const SCLineSegment2d& cutter, double eps)
    {
        return ComputePolygonCut(polygon, cutter, eps);
    }

    SCPolygonBuildResult2d BuildMultiPolygonByLinesWithDiagnostics(const SCMultiPolyline2d& polylines,
                                                                   double eps,
                                                                   SCPolygonBuildOptions2d options)
    {
        SCPolygonBuildResult2d result;
        if (!(eps > 0.0) || !std::isfinite(eps))
        {
            return result;
        }
        std::vector<RawSegment> rawSegments = CollectRawSegments(polylines, eps);
        result.diagnostics = AnalyzeBuildInput(polylines, eps);
        if (rawSegments.empty())
        {
            return result;
        }

        rawSegments = RemoveDuplicateSegments(rawSegments, eps);
        const auto rebuild = [&]() -> SCMultiPolygon2d
        {
            std::vector<RawSegment> splitSegments = SubdivideRawSegments(rawSegments, eps);
            if (splitSegments.empty())
            {
                return {};
            }

            const double repairTol = ComputeRepairTolerance(splitSegments, eps);
            if (options.allowFakeEdges && options.autoClose)
            {
                const std::size_t before = splitSegments.size();
                AutoCloseDanglingEndpoints(splitSegments, repairTol, eps);
                result.usedAutoClose = splitSegments.size() > before;
            }
            if (options.allowFakeEdges && options.autoExtend)
            {
                const std::size_t before = splitSegments.size();
                AutoExtendDanglingEndpoints(splitSegments, repairTol, eps);
                result.usedAutoExtend = splitSegments.size() > before;
            }
            splitSegments = SubdivideRawSegments(splitSegments, eps);
            splitSegments = RemoveDuplicateSegments(splitSegments, eps);
            splitSegments = PruneDanglingSegments(std::move(splitSegments), eps);
            if (splitSegments.empty())
            {
                return {};
            }
            std::vector<SCPoint2d> vertices;
            std::vector<DirectedEdge> edges;
            std::vector<std::vector<std::size_t>> outgoing;
            std::unordered_set<std::uint64_t> edgeKeys;
            AppendSplitEdges(splitSegments, vertices, edges, outgoing, edgeKeys, eps);
            if (edges.empty())
            {
                return {};
            }

            Detail::SortOutgoingFans(edges, outgoing);
            const std::vector<RingCandidate> rings =
                ExtractCandidateRings(edges, vertices, outgoing, repairTol, eps);
            return BuildFilledPolygonsFromCandidateRings(rings, eps);
        };
        result.polygons = rebuild();
        if (result.polygons.IsEmpty() && options.removeBranches && options.allowFakeEdges &&
            result.diagnostics.branchVertexCount > 0U)
        {
            std::vector<RawSegment> cleaned = PruneBranchLeaves(rawSegments, eps);
            if (!cleaned.empty() && cleaned.size() < rawSegments.size())
            {
                options.removeBranches = false;
                SCPolygonBuildResult2d retry = BuildMultiPolygonByLinesWithDiagnostics(
                    SegmentPolylines(cleaned), eps, options);
                if (retry.polygons.IsEmpty() && options.autoClose)
                {
                    const VertexGraph2d graph = BuildVertexGraph(cleaned, eps);
                    std::vector<SCPoint2d> endpoints;
                    for (std::size_t i = 0; i < graph.vertices.size(); ++i)
                    {
                        if (graph.degrees[i] == 1U) endpoints.push_back(graph.vertices[i]);
                    }
                    if (endpoints.size() == 2U && !endpoints[0].AlmostEquals(endpoints[1], eps))
                    {
                        cleaned.push_back({endpoints[0], endpoints[1], true});
                        retry = BuildMultiPolygonByLinesWithDiagnostics(SegmentPolylines(cleaned), eps, options);
                        retry.usedAutoClose = !retry.polygons.IsEmpty();
                    }
                }
                if (!retry.polygons.IsEmpty())
                {
                    result.polygons = std::move(retry.polygons);
                    result.usedAutoClose = retry.usedAutoClose;
                    result.usedAutoExtend = retry.usedAutoExtend;
                    result.usedBranchCleanup = true;
                }
            }
        }
        return result;
    }

    SCMultiPolygon2d BuildMultiPolygonByLines(const SCMultiPolyline2d& polylines, double eps)
    {
        return BuildMultiPolygonByLinesWithDiagnostics(polylines, eps).polygons;
    }

    SCPolygon2d NormalizePolygonByLines(const SCPolygon2d& polygon, double eps)
    {
        if (!(eps > 0.0) || !std::isfinite(eps))
        {
            return {};
        }
        SCMultiPolyline2d boundaries;
        AppendPolygonBoundaries(polygon, boundaries, eps);
        if (boundaries.IsEmpty())
        {
            return {};
        }

        const std::vector<double> epsCandidates{eps,
                                                std::max(eps, Geometry::Detail::kRebuildFallbackEpsilon2d),
                                                std::max(eps, 1e-7)};

        SCPolygon2d rebuilt = RebuildLargestPolygonWithCandidates(boundaries, epsCandidates);
        if (rebuilt.IsValid(eps))
        {
            return rebuilt;
        }

        const double simplifyEps =
            std::max(Geometry::Detail::kAreaEpsilon2d, 8.0 * std::max(eps, Geometry::Detail::kComparisonEpsilon2d));
        SCMultiPolyline2d simplifiedBoundaries = SimplifyBoundaryPolylines(boundaries, simplifyEps);
        if (simplifiedBoundaries.IsEmpty())
        {
            return {};
        }

        rebuilt = RebuildLargestPolygonWithCandidates(simplifiedBoundaries, epsCandidates);
        if (rebuilt.IsValid(eps))
        {
            return rebuilt;
        }

        return {};
    }
}  // namespace Geometry
