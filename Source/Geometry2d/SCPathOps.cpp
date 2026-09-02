#include "Geometry2d/SCPathOps.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../Detail/ArrangementVertices2d.h"
#include "../Detail/DirectedEdgeFans2d.h"
#include "../Detail/PolygonNesting2d.h"
#include "../Detail/RingVertices2d.h"
#include "../Detail/SegmentSubdivision2d.h"
#include "Brep/Topology.h"
#include "Core/Boolean.h"
#include "Core/Editing.h"
#include "Core/Intersection.h"
#include "Core/Projection.h"
#include "Core/Relation.h"
#include "Core/ShapeOps.h"
#include "Core/Validation.h"
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

        [[nodiscard]] double SignedSide(const SCPoint2d& point, const SCLineSegment2d& line)
        {
            return Cross(line.endPoint - line.startPoint, point - line.startPoint);
        }

        [[nodiscard]] SCPoint2d IntersectInfiniteLines(const SCPoint2d& a1,
                                                     const SCPoint2d& a2,
                                                     const SCPoint2d& b1,
                                                     const SCPoint2d& b2)
        {
            const SCVector2d r = a2 - a1;
            const SCVector2d s = b2 - b1;
            const double denom = Cross(r, s);
            if (std::abs(denom) <= Geometry::kPathOpsDefaultEpsilon)
            {
                return a2;
            }
            const double t = Cross(b1 - a1, s) / denom;
            return a1 + r * t;
        }

        [[nodiscard]] SCPolyline2d ClipPolylineLength(const SCPolyline2d& polyline, double startLength, double endLength)
        {
            if (!polyline.IsValid() || endLength < startLength)
            {
                return {};
            }

            const double total = polyline.Length();
            startLength = std::clamp(startLength, 0.0, total);
            endLength = std::clamp(endLength, 0.0, total);
            if (endLength < startLength + Geometry::kPathOpsDefaultEpsilon)
            {
                return {};
            }

            std::vector<SCPoint2d> points;
            double cursor = 0.0;
            for (std::size_t i = 0; i + 1 < polyline.PointCount(); ++i)
            {
                const SCLineSegment2d segment(polyline.PointAt(i), polyline.PointAt(i + 1));
                const double segLen = segment.Length();
                const double segStart = cursor;
                const double segEnd = cursor + segLen;

                if (segEnd < startLength - Geometry::kPathOpsDefaultEpsilon)
                {
                    cursor = segEnd;
                    continue;
                }
                if (segStart > endLength + Geometry::kPathOpsDefaultEpsilon)
                {
                    break;
                }

                const double localStart = std::max(0.0, startLength - segStart);
                const double localEnd = std::min(segLen, endLength - segStart);
                const SCPoint2d startPoint = segment.PointAtLength(localStart, true);
                const SCPoint2d endPoint = segment.PointAtLength(localEnd, true);
                if (points.empty() || !points.back().AlmostEquals(startPoint, Geometry::kPathOpsDefaultEpsilon))
                {
                    points.push_back(startPoint);
                }
                if (points.empty() || !points.back().AlmostEquals(endPoint, Geometry::kPathOpsDefaultEpsilon))
                {
                    points.push_back(endPoint);
                }

                cursor = segEnd;
            }

            return SCPolyline2d(std::move(points), SCPolylineClosure::Open);
        }

        [[nodiscard]] SCPolygon2d ClipPolygonHalfPlane(const SCPolygon2d& polygon,
                                                     const SCLineSegment2d& cutter,
                                                     bool keepLeft)
        {
            if (!polygon.IsValid() || polygon.HoleCount() != 0)
            {
                return {};
            }

            std::vector<SCPoint2d> input;
            input.reserve(polygon.OuterRing().PointCount());
            for (std::size_t i = 0; i < polygon.OuterRing().PointCount(); ++i)
            {
                input.push_back(polygon.OuterRing().PointAt(i));
            }

            std::vector<SCPoint2d> output;
            if (input.empty())
            {
                return {};
            }

            SCPoint2d prev = input.back();
            double prevSide = SignedSide(prev, cutter);
            for (const SCPoint2d& current : input)
            {
                const double currentSide = SignedSide(current, cutter);
                const bool prevInside = keepLeft ? prevSide >= -Geometry::kPathOpsDefaultEpsilon
                                                 : prevSide <= Geometry::kPathOpsDefaultEpsilon;
                const bool currentInside = keepLeft ? currentSide >= -Geometry::kPathOpsDefaultEpsilon
                                                    : currentSide <= Geometry::kPathOpsDefaultEpsilon;

                if (currentInside)
                {
                    if (!prevInside)
                    {
                        output.push_back(IntersectInfiniteLines(prev, current, cutter.startPoint, cutter.endPoint));
                    }
                    output.push_back(current);
                } else if (prevInside)
                {
                    output.push_back(IntersectInfiniteLines(prev, current, cutter.startPoint, cutter.endPoint));
                }

                prev = current;
                prevSide = currentSide;
            }

            if (output.size() < 3)
            {
                return {};
            }

            return SCPolygon2d(SCPolyline2d(std::move(output), SCPolylineClosure::Closed));
        }

        [[nodiscard]] SCPolygon2d BuildHalfPlaneClipPolygon(const SCPolygon2d& polygon,
                                                          const SCLineSegment2d& cutter,
                                                          bool keepLeft,
                                                          double eps)
        {
            if (!polygon.IsValid())
            {
                return {};
            }

            const SCBox2d bounds = polygon.Bounds();
            if (!bounds.IsValid())
            {
                return {};
            }

            const SCVector2d direction = cutter.endPoint - cutter.startPoint;
            const double length = direction.Length();
            if (length <= eps)
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
            const double diagonal = std::sqrt((maxPoint.x - minPoint.x) * (maxPoint.x - minPoint.x) +
                                              (maxPoint.y - minPoint.y) * (maxPoint.y - minPoint.y));
            const double extent = std::max(1.0, 8.0 * diagonal + 1.0);

            const SCVector2d along = tangent * extent;
            const SCVector2d side = normal * (2.0 * extent);
            const SCPoint2d anchor = cutter.startPoint;
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
            if (!clip.IsValid() || !Validate(clip, eps).valid)
            {
                return {};
            }
            return clip;
        }

        void CollectRawSegments(const SCPolyline2d& polyline, std::vector<RawSegment>& segments, double eps)
        {
            const SCPolyline2d normalized = Normalize(polyline, eps);
            if (!normalized.IsValid() || normalized.PointCount() < 2)
            {
                return;
            }

            const std::size_t segmentCount =
                normalized.IsClosed() ? normalized.PointCount() : normalized.PointCount() - 1;
            for (std::size_t i = 0; i < segmentCount; ++i)
            {
                const SCPoint2d start = normalized.PointAt(i);
                const SCPoint2d end = normalized.PointAt((i + 1) % normalized.PointCount());
                if (!start.AlmostEquals(end, eps))
                {
                    segments.push_back(RawSegment{start, end, false});
                }
            }
        }

        void AppendPolygonBoundaries(const SCPolygon2d& polygon, SCMultiPolyline2d& boundaries)
        {
            if (!polygon.OuterRing().IsValid())
            {
                return;
            }

            boundaries.Add(polygon.OuterRing());
            for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
            {
                boundaries.Add(polygon.HoleAt(i));
            }
        }

        [[nodiscard]] SCPolygon2d SelectLargestValidPolygon(const SCMultiPolygon2d& polygons)
        {
            SCPolygon2d best;
            double bestArea = 0.0;
            for (std::size_t i = 0; i < polygons.Count(); ++i)
            {
                if (!polygons[i].IsValid())
                {
                    continue;
                }

                const double candidateArea = std::abs(polygons[i].Area());
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
                if (rebuilt.Count() == 1 && rebuilt[0].IsValid())
                {
                    return rebuilt[0];
                }

                SCPolygon2d best = SelectLargestValidPolygon(rebuilt);
                if (best.IsValid())
                {
                    return best;
                }
            }

            return {};
        }

        [[nodiscard]] SCMultiPolyline2d SimplifyBoundaryPolylines(const SCMultiPolyline2d& boundaries, double eps)
        {
            SCMultiPolyline2d simplifiedBoundaries;
            for (std::size_t boundaryIndex = 0; boundaryIndex < boundaries.Count(); ++boundaryIndex)
            {
                const SCPolyline2d boundary = boundaries[boundaryIndex];
                std::vector<SCPoint2d> points;
                points.reserve(boundary.PointCount());
                for (std::size_t pointIndex = 0; pointIndex < boundary.PointCount(); ++pointIndex)
                {
                    points.push_back(boundary.PointAt(pointIndex));
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
                if (simplifiedBoundary.IsValid())
                {
                    simplifiedBoundaries.Add(std::move(simplifiedBoundary));
                }
            }

            return simplifiedBoundaries;
        }

        [[nodiscard]] std::vector<RawSegment> CollectRawSegments(const SCMultiPolyline2d& polylines, double eps)
        {
            std::vector<RawSegment> segments;
            for (std::size_t i = 0; i < polylines.Count(); ++i)
            {
                CollectRawSegments(polylines[i], segments, eps);
            }
            return segments;
        }

        [[nodiscard]] double ComputeParameterTolerance(const SCLineSegment2d& segment, double eps)
        {
            const double length = segment.Length();
            if (length <= eps)
            {
                return Geometry::kPathOpsComparisonEpsilon;
            }

            return std::min(1e-4, std::max(Geometry::kPathOpsComparisonEpsilon, 8.0 * eps / length));
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
                [eps](const SCLineSegment2d& segment) { return ComputeParameterTolerance(segment, eps); });

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
                    if (projection.distanceSquared > repairTol * repairTol || projection.distanceSquared <= eps * eps)
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

        [[nodiscard]] double ComputeAreaTolerance(const std::vector<RawSegment>& segments, double repairTol, double eps)
        {
            if (segments.empty())
            {
                return 256.0 * eps * eps;
            }

            double minX = segments.front().start.x;
            double minY = segments.front().start.y;
            double maxX = minX;
            double maxY = minY;
            for (const RawSegment& segment : segments)
            {
                minX = std::min({minX, segment.start.x, segment.end.x});
                minY = std::min({minY, segment.start.y, segment.end.y});
                maxX = std::max({maxX, segment.start.x, segment.end.x});
                maxY = std::max({maxY, segment.start.y, segment.end.y});
            }

            const double dx = maxX - minX;
            const double dy = maxY - minY;
            const double diagonal = std::sqrt(dx * dx + dy * dy);
            return std::max(256.0 * eps * eps, diagonal * std::max(repairTol, 16.0 * eps) * 1e-3);
        }

        [[nodiscard]] bool RejectRingCandidate(const RingCandidate& candidate,
                                               double repairTol,
                                               double areaTol,
                                               double eps)
        {
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
            double areaTol,
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
                if (Orientation(ring) != SCRingOrientation2d::CounterClockwise)
                {
                    continue;
                }

                const double area = std::abs(SCPolygon2d(ring).Area());
                const double score = ComputeRingCandidateScore(
                    area, syntheticPerimeter, branchVertexCount, syntheticBranchVertexCount, repairTol, eps);
                const RingCandidate candidate{
                    ring, area, perimeter, syntheticPerimeter, branchVertexCount, syntheticBranchVertexCount, score};
                if (RejectRingCandidate(candidate, repairTol, areaTol, eps))
                {
                    continue;
                }

                rings.push_back(candidate);
            }

            std::stable_sort(rings.begin(), rings.end(), [](const RingCandidate& lhs, const RingCandidate& rhs) {
                if (std::abs(lhs.score - rhs.score) > Geometry::kPathOpsComparisonEpsilon)
                {
                    return lhs.score > rhs.score;
                }
                if (std::abs(lhs.area - rhs.area) > Geometry::kPathOpsComparisonEpsilon)
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
                if (polygon.IsValid())
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
                if (Orientation(outerRing) != SCRingOrientation2d::CounterClockwise)
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
                    if (Orientation(holeRing) != SCRingOrientation2d::Clockwise)
                    {
                        holeRing = Reverse(holeRing);
                    }
                    holes.push_back(std::move(holeRing));
                }

                SCPolygon2d polygon(outerRing, std::move(holes));
                if (polygon.IsValid() && Validate(polygon, eps).valid)
                {
                    result.Add(std::move(polygon));
                }
            }

            return result;
        }
    }  // namespace

    SCPolyline2d SubPolyline(const SCPolyline2d& polyline, double startLength, double endLength)
    {
        return ClipPolylineLength(polyline, startLength, endLength);
    }

    SCPolygonCutResult2d CutPolygon(const SCPolygon2d& polygon, const SCLineSegment2d& cutter, double eps)
    {
        SCPolygonCutResult2d result;

        if (!polygon.IsValid())
        {
            return result;
        }

        if (polygon.HoleCount() > 0)
        {
            const SCPolygon2d leftClip = BuildHalfPlaneClipPolygon(polygon, cutter, true, eps);
            const SCPolygon2d rightClip = BuildHalfPlaneClipPolygon(polygon, cutter, false, eps);
            if (!leftClip.IsValid() || !rightClip.IsValid())
            {
                return result;
            }

            result.left = Intersect(polygon, leftClip, eps);
            result.right = Intersect(polygon, rightClip, eps);
            result.success = !result.left.IsEmpty() && !result.right.IsEmpty();
            return result;
        }

        const SCPolygon2d left = ClipPolygonHalfPlane(polygon, cutter, true);
        const SCPolygon2d right = ClipPolygonHalfPlane(polygon, cutter, false);
        if (!left.IsValid() || !right.IsValid())
        {
            return result;
        }
        result.success = true;
        result.left.Add(SCPolygon2d(left));
        result.right.Add(SCPolygon2d(right));
        return result;
    }

    SCMultiPolygon2d BuildMultiPolygonByLines(const SCMultiPolyline2d& polylines, double eps)
    {
        std::vector<RawSegment> rawSegments = CollectRawSegments(polylines, eps);
        if (rawSegments.empty())
        {
            return {};
        }

        rawSegments = RemoveDuplicateSegments(rawSegments, eps);
        std::vector<RawSegment> splitSegments = SubdivideRawSegments(rawSegments, eps);
        if (splitSegments.empty())
        {
            return {};
        }

        const double repairTol = ComputeRepairTolerance(splitSegments, eps);
        AutoCloseDanglingEndpoints(splitSegments, repairTol, eps);
        AutoExtendDanglingEndpoints(splitSegments, repairTol, eps);
        splitSegments = SubdivideRawSegments(splitSegments, eps);
        splitSegments = RemoveDuplicateSegments(splitSegments, eps);
        splitSegments = PruneDanglingSegments(std::move(splitSegments), eps);
        if (splitSegments.empty())
        {
            return {};
        }
        const double areaTol = ComputeAreaTolerance(splitSegments, repairTol, eps);

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
            ExtractCandidateRings(edges, vertices, outgoing, repairTol, areaTol, eps);
        return BuildFilledPolygonsFromCandidateRings(rings, eps);
    }

    SCPolygon2d NormalizePolygonByLines(const SCPolygon2d& polygon, double eps)
    {
        SCMultiPolyline2d boundaries;
        AppendPolygonBoundaries(polygon, boundaries);
        if (boundaries.IsEmpty())
        {
            return {};
        }

        const std::vector<double> epsCandidates{std::max(eps, Geometry::kPathOpsComparisonEpsilon),
                                                std::max(eps, Geometry::kPathOpsRebuildFallbackEpsilon),
                                                std::max(eps, 1e-7)};

        SCPolygon2d rebuilt = RebuildLargestPolygonWithCandidates(boundaries, epsCandidates);
        if (rebuilt.IsValid())
        {
            return rebuilt;
        }

        const double simplifyEps =
            std::max(Geometry::kPathOpsAreaEpsilon, 8.0 * std::max(eps, Geometry::kPathOpsComparisonEpsilon));
        SCMultiPolyline2d simplifiedBoundaries = SimplifyBoundaryPolylines(boundaries, simplifyEps);
        if (simplifiedBoundaries.IsEmpty())
        {
            return {};
        }

        rebuilt = RebuildLargestPolygonWithCandidates(simplifiedBoundaries, epsCandidates);
        if (rebuilt.IsValid())
        {
            return rebuilt;
        }

        return {};
    }
}  // namespace Geometry
