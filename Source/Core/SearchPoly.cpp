#include "Core/SearchPoly.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "Core/Metrics.h"
#include "Core/Relation.h"
#include "Core/ShapeOps.h"
#include "Geometry2d/SCPathOps.h"
#include "Support/Epsilon.h"
#include "Support/Geometry2d/Predicate2.h"

namespace Geometry
{
    namespace
    {
        struct GraphVertex2d
        {
            SCPoint2d point{};
            std::size_t degree{0};
        };

        struct LineNetworkAnalysis2d
        {
            SCSearchPolyDiagnostics2d diagnostics{};
            std::vector<GraphVertex2d> vertices{};
            std::vector<SCLineSegment2d> segments{};
            double repairTolerance{0.0};
        };

        struct CandidateMetrics2d
        {
            double inferredSyntheticPerimeter{0.0};
            std::size_t inferredSyntheticEdgeCount{0};
            std::size_t branchVertexCount{0};
            std::size_t syntheticBranchVertexCount{0};
            std::vector<SCLineSegment2d> inferredSyntheticEdges{};
            std::vector<SCSearchPolySyntheticEdgeKind2d> inferredSyntheticEdgeKinds{};
            std::vector<SCSearchPolySyntheticEdgeSource2d> inferredSyntheticEdgeSources{};
            std::vector<std::size_t> inferredSyntheticEdgeStartVertexIndices{};
            std::vector<std::size_t> inferredSyntheticEdgeEndVertexIndices{};
            std::vector<std::size_t> inferredSyntheticEdgeStartDegrees{};
            std::vector<std::size_t> inferredSyntheticEdgeEndDegrees{};
            std::vector<std::size_t> inferredSyntheticEdgeDanglingTouchCounts{};
            std::vector<std::size_t> inferredSyntheticEdgeBranchTouchCounts{};
            std::vector<double> inferredSyntheticEdgeLengths{};
            double branchScore{0.0};
        };

        [[nodiscard]] std::size_t FindOrAddVertex(std::vector<GraphVertex2d>& vertices,
                                                  const SCPoint2d& point,
                                                  double epsilon);

        [[nodiscard]] std::size_t BranchDegreeAtPoint(const std::vector<GraphVertex2d>& vertices,
                                                      const SCPoint2d& point,
                                                      double epsilon);

        [[nodiscard]] std::vector<GraphVertex2d> BuildSegmentVertices(const std::vector<SCLineSegment2d>& segments,
                                                                      double epsilon)
        {
            std::vector<GraphVertex2d> vertices;
            for (const SCLineSegment2d& segment : segments)
            {
                if (!segment.IsValid() || segment.startPoint.AlmostEquals(segment.endPoint, epsilon))
                {
                    continue;
                }

                const std::size_t startIndex = FindOrAddVertex(vertices, segment.startPoint, epsilon);
                const std::size_t endIndex = FindOrAddVertex(vertices, segment.endPoint, epsilon);
                if (startIndex == endIndex)
                {
                    continue;
                }

                ++vertices[startIndex].degree;
                ++vertices[endIndex].degree;
            }

            return vertices;
        }

        [[nodiscard]] std::size_t FindOrAddVertex(std::vector<GraphVertex2d>& vertices,
                                                  const SCPoint2d& point,
                                                  double epsilon)
        {
            for (std::size_t index = 0; index < vertices.size(); ++index)
            {
                if (vertices[index].point.AlmostEquals(point, epsilon))
                {
                    return index;
                }
            }

            vertices.push_back(GraphVertex2d{point, 0});
            return vertices.size() - 1;
        }

        [[nodiscard]] double ComputeRepairToleranceFromSegments(const std::vector<SCLineSegment2d>& segments,
                                                                double epsilon)
        {
            double totalLength = 0.0;
            double maxLength = 0.0;
            std::size_t counted = 0;
            for (const SCLineSegment2d& segment : segments)
            {
                const double length = segment.Length();
                if (length <= epsilon)
                {
                    continue;
                }

                totalLength += length;
                maxLength = std::max(maxLength, length);
                ++counted;
            }

            if (counted == 0)
            {
                return 16.0 * epsilon;
            }

            const double averageLength = totalLength / static_cast<double>(counted);
            return std::max(16.0 * epsilon, std::min(0.2 * averageLength, 0.25 * maxLength));
        }

        [[nodiscard]] LineNetworkAnalysis2d AnalyzeLineNetwork(const SCMultiPolyline2d& lines, double epsilon)
        {
            LineNetworkAnalysis2d analysis;
            analysis.diagnostics.inputPolylineCount = lines.Count();

            for (std::size_t lineIndex = 0; lineIndex < lines.Count(); ++lineIndex)
            {
                const SCPolyline2d& line = lines[lineIndex];
                if (!line.IsValid() || line.PointCount() < 2)
                {
                    continue;
                }

                const std::size_t segmentCount = line.IsClosed() ? line.PointCount() : line.PointCount() - 1;
                for (std::size_t segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
                {
                    const SCPoint2d start = line.PointAt(segmentIndex);
                    const SCPoint2d end = line.PointAt((segmentIndex + 1) % line.PointCount());
                    if (start.AlmostEquals(end, epsilon))
                    {
                        continue;
                    }

                    ++analysis.diagnostics.inputSegmentCount;
                    analysis.segments.push_back(SCLineSegment2d(start, end));
                    const std::size_t from = FindOrAddVertex(analysis.vertices, start, epsilon);
                    const std::size_t to = FindOrAddVertex(analysis.vertices, end, epsilon);
                    if (from != to)
                    {
                        ++analysis.vertices[from].degree;
                        ++analysis.vertices[to].degree;
                    }
                }
            }

            analysis.diagnostics.uniqueVertexCount = analysis.vertices.size();
            for (const GraphVertex2d& vertex : analysis.vertices)
            {
                if (vertex.degree == 1)
                {
                    ++analysis.diagnostics.danglingEndpointCount;
                } else if (vertex.degree > 2)
                {
                    ++analysis.diagnostics.branchVertexCount;
                }
            }

            analysis.diagnostics.inferredSyntheticEdgeCount = analysis.diagnostics.danglingEndpointCount / 2U;
            analysis.repairTolerance = ComputeRepairToleranceFromSegments(analysis.segments, epsilon);
            return analysis;
        }

        [[nodiscard]] double ComputeBranchScore(double area,
                                                double inferredSyntheticPerimeter,
                                                std::size_t branchVertexCount,
                                                std::size_t syntheticBranchVertexCount,
                                                double repairTolerance,
                                                double epsilon)
        {
            const double scale = std::max(repairTolerance, 16.0 * epsilon);
            const double syntheticPenalty = inferredSyntheticPerimeter * scale;
            const double branchPenalty = static_cast<double>(branchVertexCount) * 0.5 * scale;
            const double syntheticBranchPenalty = static_cast<double>(syntheticBranchVertexCount) * 1.5 * scale;
            return area - syntheticPenalty - branchPenalty - syntheticBranchPenalty;
        }

        [[nodiscard]] std::size_t BranchDegreeAtPoint(const std::vector<GraphVertex2d>& vertices,
                                                      const SCPoint2d& point,
                                                      double epsilon)
        {
            for (const GraphVertex2d& vertex : vertices)
            {
                if (vertex.point.AlmostEquals(point, epsilon))
                {
                    return vertex.degree;
                }
            }
            return 0;
        }

        [[nodiscard]] std::size_t FindVertexIndex(const std::vector<GraphVertex2d>& vertices,
                                                  const SCPoint2d& point,
                                                  double epsilon)
        {
            for (std::size_t index = 0; index < vertices.size(); ++index)
            {
                if (vertices[index].point.AlmostEquals(point, epsilon))
                {
                    return index;
                }
            }
            return vertices.size();
        }

        [[nodiscard]] SCSearchPolySyntheticEdgeKind2d ClassifySyntheticEdgeKind(const LineNetworkAnalysis2d& analysis,
                                                                              const SCPoint2d& start,
                                                                              const SCPoint2d& end,
                                                                              double epsilon)
        {
            const std::size_t startDegree = BranchDegreeAtPoint(analysis.vertices, start, epsilon);
            const std::size_t endDegree = BranchDegreeAtPoint(analysis.vertices, end, epsilon);
            const bool touchesDangling = startDegree == 1U || endDegree == 1U;
            const bool touchesBranch = startDegree > 2U || endDegree > 2U;

            if (touchesDangling && touchesBranch)
            {
                return SCSearchPolySyntheticEdgeKind2d::Mixed;
            }
            if (touchesDangling)
            {
                return SCSearchPolySyntheticEdgeKind2d::GapClosure;
            }
            if (touchesBranch)
            {
                return SCSearchPolySyntheticEdgeKind2d::BranchCleanup;
            }
            return SCSearchPolySyntheticEdgeKind2d::Unknown;
        }

        [[nodiscard]] SCSearchPolySyntheticEdgeKind2d DetermineDominantSyntheticEdgeKind(
            const std::vector<SCSearchPolySyntheticEdgeKind2d>& syntheticEdgeKinds)
        {
            if (syntheticEdgeKinds.empty())
            {
                return SCSearchPolySyntheticEdgeKind2d::Unknown;
            }

            std::size_t gapClosureCount = 0U;
            std::size_t branchCleanupCount = 0U;
            std::size_t mixedCount = 0U;
            std::size_t unknownCount = 0U;
            for (const SCSearchPolySyntheticEdgeKind2d kind : syntheticEdgeKinds)
            {
                switch (kind)
                {
                    case SCSearchPolySyntheticEdgeKind2d::GapClosure:
                        ++gapClosureCount;
                        break;
                    case SCSearchPolySyntheticEdgeKind2d::BranchCleanup:
                        ++branchCleanupCount;
                        break;
                    case SCSearchPolySyntheticEdgeKind2d::Mixed:
                        ++mixedCount;
                        break;
                    default:
                        ++unknownCount;
                        break;
                }
            }

            const std::size_t maxCount =
                std::max(std::max(gapClosureCount, branchCleanupCount), std::max(mixedCount, unknownCount));
            const std::size_t winnerCount = (gapClosureCount == maxCount ? 1U : 0U) +
                                            (branchCleanupCount == maxCount ? 1U : 0U) +
                                            (mixedCount == maxCount ? 1U : 0U) + (unknownCount == maxCount ? 1U : 0U);
            if (winnerCount != 1U)
            {
                return SCSearchPolySyntheticEdgeKind2d::Mixed;
            }
            if (mixedCount == maxCount)
            {
                return SCSearchPolySyntheticEdgeKind2d::Mixed;
            }
            if (gapClosureCount == maxCount)
            {
                return SCSearchPolySyntheticEdgeKind2d::GapClosure;
            }
            if (branchCleanupCount == maxCount)
            {
                return SCSearchPolySyntheticEdgeKind2d::BranchCleanup;
            }
            return SCSearchPolySyntheticEdgeKind2d::Unknown;
        }

        [[nodiscard]] SCSearchPolySyntheticEdgeSource2d DetermineDominantSyntheticEdgeSource(
            const std::vector<SCSearchPolySyntheticEdgeSource2d>& syntheticEdgeSources)
        {
            if (syntheticEdgeSources.empty())
            {
                return SCSearchPolySyntheticEdgeSource2d::Unknown;
            }

            std::size_t singleGapCloseCount = 0U;
            std::size_t branchCleanupCount = 0U;
            std::size_t mixedBridgeCount = 0U;
            std::size_t unknownCount = 0U;
            for (const SCSearchPolySyntheticEdgeSource2d source : syntheticEdgeSources)
            {
                switch (source)
                {
                    case SCSearchPolySyntheticEdgeSource2d::SingleGapClose:
                        ++singleGapCloseCount;
                        break;
                    case SCSearchPolySyntheticEdgeSource2d::BranchCleanup:
                        ++branchCleanupCount;
                        break;
                    case SCSearchPolySyntheticEdgeSource2d::MixedBridge:
                        ++mixedBridgeCount;
                        break;
                    default:
                        ++unknownCount;
                        break;
                }
            }

            const std::size_t maxCount =
                std::max(std::max(singleGapCloseCount, branchCleanupCount), std::max(mixedBridgeCount, unknownCount));
            const std::size_t winnerCount =
                (singleGapCloseCount == maxCount ? 1U : 0U) + (branchCleanupCount == maxCount ? 1U : 0U) +
                (mixedBridgeCount == maxCount ? 1U : 0U) + (unknownCount == maxCount ? 1U : 0U);
            if (winnerCount != 1U)
            {
                return SCSearchPolySyntheticEdgeSource2d::MixedBridge;
            }
            if (mixedBridgeCount == maxCount)
            {
                return SCSearchPolySyntheticEdgeSource2d::MixedBridge;
            }
            if (singleGapCloseCount == maxCount)
            {
                return SCSearchPolySyntheticEdgeSource2d::SingleGapClose;
            }
            if (branchCleanupCount == maxCount)
            {
                return SCSearchPolySyntheticEdgeSource2d::BranchCleanup;
            }
            return SCSearchPolySyntheticEdgeSource2d::Unknown;
        }

        [[nodiscard]] SCSearchPolyPenaltyKind2d SummarizePenaltyKinds(
            const std::vector<SCSearchPolyPenaltyKind2d>& penaltyKinds)
        {
            if (penaltyKinds.empty())
            {
                return SCSearchPolyPenaltyKind2d::None;
            }

            const SCSearchPolyPenaltyKind2d first = penaltyKinds.front();
            for (const SCSearchPolyPenaltyKind2d candidate : penaltyKinds)
            {
                if (candidate != first)
                {
                    return SCSearchPolyPenaltyKind2d::Mixed;
                }
            }
            return first;
        }

        [[nodiscard]] SCSearchPolySyntheticEdgeSource2d ClassifySyntheticEdgeSource(
            SCSearchPolySyntheticEdgeKind2d syntheticEdgeKind)
        {
            switch (syntheticEdgeKind)
            {
                case SCSearchPolySyntheticEdgeKind2d::GapClosure:
                    return SCSearchPolySyntheticEdgeSource2d::SingleGapClose;
                case SCSearchPolySyntheticEdgeKind2d::BranchCleanup:
                    return SCSearchPolySyntheticEdgeSource2d::BranchCleanup;
                case SCSearchPolySyntheticEdgeKind2d::Mixed:
                    return SCSearchPolySyntheticEdgeSource2d::MixedBridge;
                default:
                    return SCSearchPolySyntheticEdgeSource2d::Unknown;
            }
        }

        [[nodiscard]] std::size_t CountDanglingTouches(std::size_t startDegree, std::size_t endDegree)
        {
            return (startDegree == 1U ? 1U : 0U) + (endDegree == 1U ? 1U : 0U);
        }

        [[nodiscard]] std::size_t CountBranchTouches(std::size_t startDegree, std::size_t endDegree)
        {
            return (startDegree > 2U ? 1U : 0U) + (endDegree > 2U ? 1U : 0U);
        }

        [[nodiscard]] bool CoversBoundaryEdge(const SCLineSegment2d& boundaryEdge,
                                              const std::vector<SCLineSegment2d>& inputSegments,
                                              double epsilon)
        {
            if (!boundaryEdge.IsValid())
            {
                return false;
            }

            const SCVector2d direction = boundaryEdge.endPoint - boundaryEdge.startPoint;
            const double length = direction.Length();
            const double lengthSquared = direction.LengthSquared();
            if (length <= epsilon || lengthSquared <= epsilon * epsilon)
            {
                return false;
            }

            std::vector<std::pair<double, double>> intervals;
            intervals.reserve(inputSegments.size());
            for (const SCLineSegment2d& inputSegment : inputSegments)
            {
                if (!inputSegment.IsValid())
                {
                    continue;
                }

                const double startOffset =
                    std::abs(Cross(direction, inputSegment.startPoint - boundaryEdge.startPoint));
                const double endOffset = std::abs(Cross(direction, inputSegment.endPoint - boundaryEdge.startPoint));
                if (startOffset > epsilon * length || endOffset > epsilon * length)
                {
                    continue;
                }

                double t0 = Dot(inputSegment.startPoint - boundaryEdge.startPoint, direction) / lengthSquared;
                double t1 = Dot(inputSegment.endPoint - boundaryEdge.startPoint, direction) / lengthSquared;
                if (t1 < t0)
                {
                    std::swap(t0, t1);
                }

                const double clippedStart = std::max(0.0, t0);
                const double clippedEnd = std::min(1.0, t1);
                if (clippedEnd <= clippedStart + epsilon / length)
                {
                    continue;
                }

                intervals.emplace_back(clippedStart, clippedEnd);
            }

            if (intervals.empty())
            {
                return false;
            }

            std::sort(intervals.begin(),
                      intervals.end(),
                      [](const std::pair<double, double>& left, const std::pair<double, double>& right) {
                          if (std::abs(left.first - right.first) > Geometry::kSearchPolyComparisonEpsilon)
                          {
                              return left.first < right.first;
                          }
                          return left.second < right.second;
                      });

            const double parameterTolerance = epsilon / length;
            double coveredUntil = 0.0;
            bool seeded = false;
            for (const auto& interval : intervals)
            {
                if (!seeded)
                {
                    if (interval.first > parameterTolerance)
                    {
                        return false;
                    }
                    coveredUntil = interval.second;
                    seeded = true;
                    continue;
                }

                if (interval.first > coveredUntil + parameterTolerance)
                {
                    return false;
                }
                coveredUntil = std::max(coveredUntil, interval.second);
            }

            return seeded && coveredUntil >= 1.0 - parameterTolerance;
        }

        [[nodiscard]] SCMultiPolyline2d BuildSegmentPolylines(const std::vector<SCLineSegment2d>& segments)
        {
            SCMultiPolyline2d polylines;
            for (const SCLineSegment2d& segment : segments)
            {
                if (!segment.IsValid())
                {
                    continue;
                }

                polylines.Add(SCPolyline2d({segment.startPoint, segment.endPoint}, SCPolylineClosure::Open));
            }
            return polylines;
        }

        [[nodiscard]] std::vector<SCLineSegment2d> PruneBranchDanglingSegments(std::vector<SCLineSegment2d> segments,
                                                                             double epsilon)
        {
            bool changed = true;
            while (changed)
            {
                changed = false;
                const std::vector<GraphVertex2d> vertices = BuildSegmentVertices(segments, epsilon);
                std::vector<SCLineSegment2d> kept;
                kept.reserve(segments.size());
                for (const SCLineSegment2d& segment : segments)
                {
                    const std::size_t startDegree = BranchDegreeAtPoint(vertices, segment.startPoint, epsilon);
                    const std::size_t endDegree = BranchDegreeAtPoint(vertices, segment.endPoint, epsilon);
                    const bool pruneStartLeaf = startDegree == 1U && endDegree > 2U;
                    const bool pruneEndLeaf = endDegree == 1U && startDegree > 2U;
                    if (pruneStartLeaf || pruneEndLeaf)
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

        [[nodiscard]] SCMultiPolygon2d TryBuildPolygonsWithBranchCleanupFallback(const LineNetworkAnalysis2d& analysis,
                                                                               const SCSearchPolyOptions2d& options)
        {
            if (!options.removeBranches || !options.allowFakeEdges || analysis.diagnostics.branchVertexCount == 0U)
            {
                return {};
            }

            std::vector<SCLineSegment2d> prunedSegments = PruneBranchDanglingSegments(analysis.segments, options.epsilon);
            if (prunedSegments.empty() || prunedSegments.size() >= analysis.segments.size())
            {
                return {};
            }

            SCMultiPolygon2d polygons = BuildMultiPolygonByLines(BuildSegmentPolylines(prunedSegments), options.epsilon);
            if (polygons.Count() > 0)
            {
                return polygons;
            }

            if (!options.autoClose)
            {
                return {};
            }

            const std::vector<GraphVertex2d> vertices = BuildSegmentVertices(prunedSegments, options.epsilon);
            std::vector<SCPoint2d> danglingEndpoints;
            for (const GraphVertex2d& vertex : vertices)
            {
                if (vertex.degree == 1U)
                {
                    danglingEndpoints.push_back(vertex.point);
                }
            }

            if (danglingEndpoints.size() != 2U ||
                danglingEndpoints[0].AlmostEquals(danglingEndpoints[1], options.epsilon))
            {
                return {};
            }

            prunedSegments.push_back(SCLineSegment2d(danglingEndpoints[0], danglingEndpoints[1]));
            return BuildMultiPolygonByLines(BuildSegmentPolylines(prunedSegments), options.epsilon);
        }

        void AccumulateRingMetrics(const SCPolyline2d& ring,
                                   const LineNetworkAnalysis2d& analysis,
                                   double epsilon,
                                   CandidateMetrics2d& metrics)
        {
            if (!ring.IsValid() || ring.PointCount() < 3)
            {
                return;
            }

            const std::size_t segmentCount = ring.PointCount();
            for (std::size_t segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
            {
                const SCPoint2d start = ring.PointAt(segmentIndex);
                const SCPoint2d end = ring.PointAt((segmentIndex + 1) % segmentCount);
                if (start.AlmostEquals(end, epsilon))
                {
                    continue;
                }

                const SCLineSegment2d boundaryEdge(start, end);
                const double edgeLength = boundaryEdge.Length();

                const bool inferredSynthetic = !CoversBoundaryEdge(boundaryEdge, analysis.segments, epsilon);
                if (inferredSynthetic)
                {
                    const std::size_t startVertexIndex = FindVertexIndex(analysis.vertices, start, epsilon);
                    const std::size_t endVertexIndex = FindVertexIndex(analysis.vertices, end, epsilon);
                    const std::size_t startDegree = BranchDegreeAtPoint(analysis.vertices, start, epsilon);
                    const std::size_t endDegree = BranchDegreeAtPoint(analysis.vertices, end, epsilon);
                    const SCSearchPolySyntheticEdgeKind2d syntheticKind =
                        ClassifySyntheticEdgeKind(analysis, start, end, epsilon);
                    ++metrics.inferredSyntheticEdgeCount;
                    metrics.inferredSyntheticPerimeter += edgeLength;
                    metrics.inferredSyntheticEdges.push_back(boundaryEdge);
                    metrics.inferredSyntheticEdgeKinds.push_back(syntheticKind);
                    metrics.inferredSyntheticEdgeSources.push_back(ClassifySyntheticEdgeSource(syntheticKind));
                    metrics.inferredSyntheticEdgeStartVertexIndices.push_back(startVertexIndex);
                    metrics.inferredSyntheticEdgeEndVertexIndices.push_back(endVertexIndex);
                    metrics.inferredSyntheticEdgeStartDegrees.push_back(startDegree);
                    metrics.inferredSyntheticEdgeEndDegrees.push_back(endDegree);
                    metrics.inferredSyntheticEdgeDanglingTouchCounts.push_back(
                        CountDanglingTouches(startDegree, endDegree));
                    metrics.inferredSyntheticEdgeBranchTouchCounts.push_back(
                        CountBranchTouches(startDegree, endDegree));
                    metrics.inferredSyntheticEdgeLengths.push_back(edgeLength);
                }

                if (BranchDegreeAtPoint(analysis.vertices, start, epsilon) > 2U)
                {
                    ++metrics.branchVertexCount;
                    if (inferredSynthetic)
                    {
                        ++metrics.syntheticBranchVertexCount;
                    }
                }
            }
        }

        [[nodiscard]] CandidateMetrics2d AnalyzeCandidate(const SCPolygon2d& polygon,
                                                          const LineNetworkAnalysis2d& analysis,
                                                          double epsilon)
        {
            CandidateMetrics2d metrics;
            AccumulateRingMetrics(polygon.OuterRing(), analysis, epsilon, metrics);
            for (std::size_t holeIndex = 0; holeIndex < polygon.HoleCount(); ++holeIndex)
            {
                AccumulateRingMetrics(polygon.HoleAt(holeIndex), analysis, epsilon, metrics);
            }

            const double polygonArea = polygon.Area();
            metrics.branchScore = ComputeBranchScore(std::abs(polygonArea),
                                                     metrics.inferredSyntheticPerimeter,
                                                     metrics.branchVertexCount,
                                                     metrics.syntheticBranchVertexCount,
                                                     analysis.repairTolerance,
                                                     epsilon);
            return metrics;
        }

        [[nodiscard]] SCSearchPolyCandidate2d MakeCandidate(const SCPolygon2d& polygon, const CandidateMetrics2d& metrics)
        {
            SCSearchPolyPenaltyKind2d dominantPenaltyKind = SCSearchPolyPenaltyKind2d::None;
            const SCSearchPolySyntheticEdgeKind2d dominantSyntheticEdgeKind =
                DetermineDominantSyntheticEdgeKind(metrics.inferredSyntheticEdgeKinds);
            const SCSearchPolySyntheticEdgeSource2d dominantSyntheticEdgeSource =
                DetermineDominantSyntheticEdgeSource(metrics.inferredSyntheticEdgeSources);
            const bool hasSynthetic = metrics.inferredSyntheticEdgeCount > 0U;
            const bool hasBranch = metrics.branchVertexCount > 0U;
            const bool hasSyntheticBranch = metrics.syntheticBranchVertexCount > 0U;
            if (hasSyntheticBranch)
            {
                dominantPenaltyKind = SCSearchPolyPenaltyKind2d::SyntheticBranchPenalty;
            } else if (hasSynthetic && hasBranch)
            {
                dominantPenaltyKind = SCSearchPolyPenaltyKind2d::Mixed;
            } else if (hasSynthetic)
            {
                dominantPenaltyKind = SCSearchPolyPenaltyKind2d::SyntheticClosure;
            } else if (hasBranch)
            {
                dominantPenaltyKind = SCSearchPolyPenaltyKind2d::BranchPenalty;
            }

            const double polygonArea = polygon.Area();
            return SCSearchPolyCandidate2d{polygon,
                                         std::abs(polygonArea),
                                         metrics.branchScore,
                                         metrics.inferredSyntheticPerimeter,
                                         polygon.HoleCount(),
                                         metrics.inferredSyntheticEdgeCount,
                                         metrics.branchVertexCount,
                                         metrics.syntheticBranchVertexCount,
                                         dominantPenaltyKind,
                                         dominantSyntheticEdgeKind,
                                         dominantSyntheticEdgeSource,
                                         metrics.inferredSyntheticEdges,
                                         metrics.inferredSyntheticEdgeKinds,
                                         metrics.inferredSyntheticEdgeSources,
                                         metrics.inferredSyntheticEdgeStartVertexIndices,
                                         metrics.inferredSyntheticEdgeEndVertexIndices,
                                         metrics.inferredSyntheticEdgeStartDegrees,
                                         metrics.inferredSyntheticEdgeEndDegrees,
                                         metrics.inferredSyntheticEdgeDanglingTouchCounts,
                                         metrics.inferredSyntheticEdgeBranchTouchCounts,
                                         metrics.inferredSyntheticEdgeLengths,
                                         0};
        }

        void RankCandidates(std::vector<SCSearchPolyCandidate2d>& candidates)
        {
            std::stable_sort(
                candidates.begin(),
                candidates.end(),
                [](const SCSearchPolyCandidate2d& left, const SCSearchPolyCandidate2d& right) {
                    if (std::abs(left.branchScore - right.branchScore) > Geometry::kSearchPolyComparisonEpsilon)
                    {
                        return left.branchScore > right.branchScore;
                    }
                    if (std::abs(left.absoluteArea - right.absoluteArea) > Geometry::kSearchPolyComparisonEpsilon)
                    {
                        return left.absoluteArea > right.absoluteArea;
                    }
                    if (left.holeCount != right.holeCount)
                    {
                        return left.holeCount < right.holeCount;
                    }
                    return left.polygon.OuterRing().PointCount() < right.polygon.OuterRing().PointCount();
                });

            for (std::size_t index = 0; index < candidates.size(); ++index)
            {
                candidates[index].rank = index;
            }
        }

        void PopulateResultExplanation(SCSearchPolyResult2d& result)
        {
            if (result.candidates.empty())
            {
                return;
            }

            const SCSearchPolyCandidate2d& bestCandidate = result.candidates.front();
            result.bestCandidateSyntheticPerimeter = bestCandidate.inferredSyntheticPerimeter;
            result.bestCandidateSyntheticEdgeCount = bestCandidate.inferredSyntheticEdgeCount;
            result.bestCandidateSyntheticEdgeKind = bestCandidate.dominantSyntheticEdgeKind;
            result.bestCandidateSyntheticEdgeSource = bestCandidate.dominantSyntheticEdgeSource;
            result.bestCandidatePenaltyKind = bestCandidate.dominantPenaltyKind;

            for (const SCSearchPolyCandidate2d& candidate : result.candidates)
            {
                if (candidate.inferredSyntheticEdgeCount > 0U)
                {
                    ++result.candidateCountWithSyntheticEdges;
                }
                if (candidate.branchVertexCount > 0U || candidate.syntheticBranchVertexCount > 0U)
                {
                    ++result.candidateCountWithBranchPenalty;
                }
            }

            result.ambiguousTopCandidateCount = 1U;
            std::vector<SCSearchPolyPenaltyKind2d> ambiguousTopPenaltyKinds{bestCandidate.dominantPenaltyKind};
            std::vector<SCSearchPolySyntheticEdgeKind2d> ambiguousTopSyntheticKinds{
                bestCandidate.dominantSyntheticEdgeKind};
            std::vector<SCSearchPolySyntheticEdgeSource2d> ambiguousTopSyntheticSources{
                bestCandidate.dominantSyntheticEdgeSource};
            for (std::size_t index = 1; index < result.candidates.size(); ++index)
            {
                const SCSearchPolyCandidate2d& candidate = result.candidates[index];
                if (std::abs(candidate.branchScore - bestCandidate.branchScore) <=
                    Geometry::kSearchPolyComparisonEpsilon)
                {
                    ++result.ambiguousTopCandidateCount;
                    ambiguousTopPenaltyKinds.push_back(candidate.dominantPenaltyKind);
                    ambiguousTopSyntheticKinds.push_back(candidate.dominantSyntheticEdgeKind);
                    ambiguousTopSyntheticSources.push_back(candidate.dominantSyntheticEdgeSource);
                    continue;
                }

                result.bestCandidateScoreMargin = bestCandidate.branchScore - candidate.branchScore;
                break;
            }
            result.ambiguousTopPenaltyKind = SummarizePenaltyKinds(ambiguousTopPenaltyKinds);
            result.ambiguousTopSyntheticEdgeKind = DetermineDominantSyntheticEdgeKind(ambiguousTopSyntheticKinds);
            result.ambiguousTopSyntheticEdgeSource = DetermineDominantSyntheticEdgeSource(ambiguousTopSyntheticSources);
            for (std::size_t index = 0; index < result.ambiguousTopCandidateCount; ++index)
            {
                const SCSearchPolyCandidate2d& candidate = result.candidates[index];
                if (candidate.inferredSyntheticEdgeCount > 0U)
                {
                    ++result.ambiguousTopCandidateCountWithSyntheticEdges;
                }
                if (candidate.branchVertexCount > 0U || candidate.syntheticBranchVertexCount > 0U)
                {
                    ++result.ambiguousTopCandidateCountWithBranchPenalty;
                }
            }

            if (result.candidates.size() >= 2U)
            {
                const SCSearchPolyCandidate2d& runnerUp = result.candidates[1];
                result.runnerUpSyntheticPerimeter = runnerUp.inferredSyntheticPerimeter;
                result.runnerUpSyntheticEdgeCount = runnerUp.inferredSyntheticEdgeCount;
                result.runnerUpSyntheticEdgeKind = runnerUp.dominantSyntheticEdgeKind;
                result.runnerUpSyntheticEdgeSource = runnerUp.dominantSyntheticEdgeSource;
                result.runnerUpBranchVertexCount = runnerUp.branchVertexCount + runnerUp.syntheticBranchVertexCount;
                result.runnerUpPenaltyKind = runnerUp.dominantPenaltyKind;
                result.bestCandidateBeatsSyntheticRunnerUp =
                    bestCandidate.inferredSyntheticEdgeCount < runnerUp.inferredSyntheticEdgeCount;
                const std::size_t bestBranchPenaltyCount =
                    bestCandidate.branchVertexCount + bestCandidate.syntheticBranchVertexCount;
                result.bestCandidateBeatsBranchRunnerUp = bestBranchPenaltyCount < result.runnerUpBranchVertexCount;
            }
        }
    }  // namespace

    SCSearchPolyResult2d SearchPolygons(const SCMultiPolyline2d& lines, SCSearchPolyOptions2d options)
    {
        SCSearchPolyResult2d result;
        if (options.epsilon <= 0.0 || lines.Count() == 0)
        {
            result.issue = SCSearchPolyIssue2d::InvalidInput;
            return result;
        }

        const LineNetworkAnalysis2d analysis = AnalyzeLineNetwork(lines, options.epsilon);
        result.diagnostics = analysis.diagnostics;
        result.polygons = BuildMultiPolygonByLines(lines, options.epsilon);
        if (result.polygons.Count() == 0)
        {
            result.polygons = TryBuildPolygonsWithBranchCleanupFallback(analysis, options);
        }
        if (result.polygons.Count() == 0)
        {
            result.issue = SCSearchPolyIssue2d::NoClosedPolygonFound;
            return result;
        }

        result.candidates.reserve(result.polygons.Count());
        for (std::size_t index = 0; index < result.polygons.Count(); ++index)
        {
            if (result.polygons[index].IsValid())
            {
                result.candidates.push_back(MakeCandidate(
                    result.polygons[index], AnalyzeCandidate(result.polygons[index], analysis, options.epsilon)));
            }
        }

        if (result.candidates.empty())
        {
            result.issue = SCSearchPolyIssue2d::NoClosedPolygonFound;
            result.polygons = {};
            return result;
        }

        RankCandidates(result.candidates);
        PopulateResultExplanation(result);

        result.usedSyntheticEdges =
            std::any_of(result.candidates.begin(), result.candidates.end(), [](const SCSearchPolyCandidate2d& candidate) {
                return candidate.inferredSyntheticEdgeCount > 0;
            });
        result.usedBranchScoring =
            std::any_of(result.candidates.begin(), result.candidates.end(), [](const SCSearchPolyCandidate2d& candidate) {
                return candidate.inferredSyntheticEdgeCount > 0 || candidate.branchVertexCount > 0 ||
                       candidate.syntheticBranchVertexCount > 0;
            });

        if (result.diagnostics.danglingEndpointCount > 0)
        {
            result.usedAutoClose = options.autoClose;
        }
        if (result.diagnostics.branchVertexCount > 0 && result.usedBranchScoring)
        {
            result.usedAutoExtend = options.autoExtend;
        }

        return result;
    }

    std::optional<SCSearchPolyCandidate2d> SearchPolygonContainingPoint(const SCMultiPolyline2d& lines,
                                                                      const SCPoint2d& point,
                                                                      SCSearchPolyOptions2d options)
    {
        const SCSearchPolyResult2d result = SearchPolygons(lines, options);
        if (!result.IsSuccess() || !point.IsValid())
        {
            return std::nullopt;
        }

        std::optional<SCSearchPolyCandidate2d> bestCandidate;
        double bestArea = std::numeric_limits<double>::infinity();
        for (const SCSearchPolyCandidate2d& candidate : result.candidates)
        {
            const SCPointContainment2d containment = LocatePoint(point, candidate.polygon, options.epsilon);
            if (containment == SCPointContainment2d::Outside)
            {
                continue;
            }

            if (!bestCandidate.has_value() || candidate.absoluteArea < bestArea)
            {
                bestArea = candidate.absoluteArea;
                bestCandidate = candidate;
            }
        }

        return bestCandidate;
    }
}  // namespace Geometry

