#include "sdk/GeometrySearchPoly.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "algorithm/Predicate2.h"
#include "sdk/GeometryMetrics.h"
#include "sdk/GeometryPathOps.h"
#include "sdk/GeometryRelation.h"
#include "sdk/GeometryShapeOps.h"

namespace geometry::sdk
{
namespace
{
struct GraphVertex2d
{
    Point2d point{};
    std::size_t degree{0};
};

struct LineNetworkAnalysis2d
{
    SearchPolyDiagnostics2d diagnostics{};
    std::vector<GraphVertex2d> vertices{};
    std::vector<LineSegment2d> segments{};
    double repairTolerance{0.0};
};

struct CandidateMetrics2d
{
    double inferredSyntheticPerimeter{0.0};
    std::size_t inferredSyntheticEdgeCount{0};
    std::size_t branchVertexCount{0};
    std::size_t syntheticBranchVertexCount{0};
    double branchScore{0.0};
};

[[nodiscard]] std::size_t FindOrAddVertex(
    std::vector<GraphVertex2d>& vertices,
    const Point2d& point,
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

[[nodiscard]] double ComputeRepairToleranceFromSegments(const std::vector<LineSegment2d>& segments, double epsilon)
{
    double totalLength = 0.0;
    double maxLength = 0.0;
    std::size_t counted = 0;
    for (const LineSegment2d& segment : segments)
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

[[nodiscard]] LineNetworkAnalysis2d AnalyzeLineNetwork(
    const MultiPolyline2d& lines,
    double epsilon)
{
    LineNetworkAnalysis2d analysis;
    analysis.diagnostics.inputPolylineCount = lines.Count();

    for (std::size_t lineIndex = 0; lineIndex < lines.Count(); ++lineIndex)
    {
        const Polyline2d& line = lines[lineIndex];
        if (!line.IsValid() || line.PointCount() < 2)
        {
            continue;
        }

        const std::size_t segmentCount = line.IsClosed() ? line.PointCount() : line.PointCount() - 1;
        for (std::size_t segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
        {
            const Point2d start = line.PointAt(segmentIndex);
            const Point2d end = line.PointAt((segmentIndex + 1) % line.PointCount());
            if (start.AlmostEquals(end, epsilon))
            {
                continue;
            }

            ++analysis.diagnostics.inputSegmentCount;
            analysis.segments.push_back(LineSegment2d(start, end));
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
        }
        else if (vertex.degree > 2)
        {
            ++analysis.diagnostics.branchVertexCount;
        }
    }

    analysis.diagnostics.inferredSyntheticEdgeCount = analysis.diagnostics.danglingEndpointCount / 2U;
    analysis.repairTolerance = ComputeRepairToleranceFromSegments(analysis.segments, epsilon);
    return analysis;
}

[[nodiscard]] double ComputeBranchScore(
    double area,
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

[[nodiscard]] std::size_t BranchDegreeAtPoint(
    const std::vector<GraphVertex2d>& vertices,
    const Point2d& point,
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

[[nodiscard]] bool CoversBoundaryEdge(
    const LineSegment2d& boundaryEdge,
    const std::vector<LineSegment2d>& inputSegments,
    double epsilon)
{
    if (!boundaryEdge.IsValid())
    {
        return false;
    }

    const Vector2d direction = boundaryEdge.endPoint - boundaryEdge.startPoint;
    const double length = direction.Length();
    const double lengthSquared = direction.LengthSquared();
    if (length <= epsilon || lengthSquared <= epsilon * epsilon)
    {
        return false;
    }

    std::vector<std::pair<double, double>> intervals;
    intervals.reserve(inputSegments.size());
    for (const LineSegment2d& inputSegment : inputSegments)
    {
        if (!inputSegment.IsValid())
        {
            continue;
        }

        const double startOffset = std::abs(Cross(direction, inputSegment.startPoint - boundaryEdge.startPoint));
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

    std::sort(intervals.begin(), intervals.end(), [](const std::pair<double, double>& left, const std::pair<double, double>& right) {
        if (std::abs(left.first - right.first) > 1e-12)
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

void AccumulateRingMetrics(
    const Polyline2d& ring,
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
        const Point2d start = ring.PointAt(segmentIndex);
        const Point2d end = ring.PointAt((segmentIndex + 1) % segmentCount);
        if (start.AlmostEquals(end, epsilon))
        {
            continue;
        }

        const LineSegment2d boundaryEdge(start, end);
        const double edgeLength = boundaryEdge.Length();

        const bool inferredSynthetic = !CoversBoundaryEdge(boundaryEdge, analysis.segments, epsilon);
        if (inferredSynthetic)
        {
            ++metrics.inferredSyntheticEdgeCount;
            metrics.inferredSyntheticPerimeter += edgeLength;
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

[[nodiscard]] CandidateMetrics2d AnalyzeCandidate(
    const Polygon2d& polygon,
    const LineNetworkAnalysis2d& analysis,
    double epsilon)
{
    CandidateMetrics2d metrics;
    AccumulateRingMetrics(polygon.OuterRing(), analysis, epsilon, metrics);
    for (std::size_t holeIndex = 0; holeIndex < polygon.HoleCount(); ++holeIndex)
    {
        AccumulateRingMetrics(polygon.HoleAt(holeIndex), analysis, epsilon, metrics);
    }

    const double polygonArea = Area(polygon);
    metrics.branchScore = ComputeBranchScore(
        std::abs(polygonArea),
        metrics.inferredSyntheticPerimeter,
        metrics.branchVertexCount,
        metrics.syntheticBranchVertexCount,
        analysis.repairTolerance,
        epsilon);
    return metrics;
}

[[nodiscard]] SearchPolyCandidate2d MakeCandidate(
    const Polygon2d& polygon,
    const CandidateMetrics2d& metrics)
{
    const double polygonArea = Area(polygon);
    return SearchPolyCandidate2d{
        polygon,
        std::abs(polygonArea),
        metrics.branchScore,
        metrics.inferredSyntheticPerimeter,
        polygon.HoleCount(),
        metrics.inferredSyntheticEdgeCount,
        metrics.branchVertexCount,
        metrics.syntheticBranchVertexCount,
        0};
}

void RankCandidates(std::vector<SearchPolyCandidate2d>& candidates)
{
    std::stable_sort(candidates.begin(), candidates.end(), [](const SearchPolyCandidate2d& left, const SearchPolyCandidate2d& right) {
        if (std::abs(left.branchScore - right.branchScore) > 1e-12)
        {
            return left.branchScore > right.branchScore;
        }
        if (std::abs(left.absoluteArea - right.absoluteArea) > 1e-12)
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
} // namespace

SearchPolyResult2d SearchPolygons(const MultiPolyline2d& lines, SearchPolyOptions2d options)
{
    SearchPolyResult2d result;
    if (options.epsilon <= 0.0 || lines.Count() == 0)
    {
        result.issue = SearchPolyIssue2d::InvalidInput;
        return result;
    }

    const LineNetworkAnalysis2d analysis = AnalyzeLineNetwork(lines, options.epsilon);
    result.diagnostics = analysis.diagnostics;
    result.polygons = BuildMultiPolygonByLines(lines, options.epsilon);
    if (result.polygons.Count() == 0)
    {
        result.issue = SearchPolyIssue2d::NoClosedPolygonFound;
        return result;
    }

    result.candidates.reserve(result.polygons.Count());
    for (std::size_t index = 0; index < result.polygons.Count(); ++index)
    {
        if (result.polygons[index].IsValid())
        {
            result.candidates.push_back(MakeCandidate(result.polygons[index], AnalyzeCandidate(result.polygons[index], analysis, options.epsilon)));
        }
    }

    if (result.candidates.empty())
    {
        result.issue = SearchPolyIssue2d::NoClosedPolygonFound;
        result.polygons = {};
        return result;
    }

    RankCandidates(result.candidates);

    result.usedSyntheticEdges =
        std::any_of(result.candidates.begin(), result.candidates.end(), [](const SearchPolyCandidate2d& candidate) {
            return candidate.inferredSyntheticEdgeCount > 0;
        });
    result.usedBranchScoring =
        std::any_of(result.candidates.begin(), result.candidates.end(), [](const SearchPolyCandidate2d& candidate) {
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

std::optional<SearchPolyCandidate2d> SearchPolygonContainingPoint(
    const MultiPolyline2d& lines,
    const Point2d& point,
    SearchPolyOptions2d options)
{
    const SearchPolyResult2d result = SearchPolygons(lines, options);
    if (!result.IsSuccess() || !point.IsValid())
    {
        return std::nullopt;
    }

    std::optional<SearchPolyCandidate2d> bestCandidate;
    double bestArea = std::numeric_limits<double>::infinity();
    for (const SearchPolyCandidate2d& candidate : result.candidates)
    {
        const PointContainment2d containment = LocatePoint(point, candidate.polygon, options.epsilon);
        if (containment == PointContainment2d::Outside)
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
} // namespace geometry::sdk
