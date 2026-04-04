#include "sdk/GeometryPathOps.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "algorithm/Predicate2.h"
#include "common/Epsilon.h"
#include "sdk/GeometryEditing.h"
#include "sdk/GeometryBoolean.h"
#include "sdk/GeometryIntersection.h"
#include "sdk/GeometryProjection.h"
#include "sdk/GeometryRelation.h"
#include "sdk/GeometryShapeOps.h"
#include "sdk/GeometryTopology.h"
#include "sdk/GeometryValidation.h"

namespace geometry::sdk
{
namespace
{
struct DirectedEdge
{
    std::size_t from{0};
    std::size_t to{0};
    std::size_t twin{0};
    double angle{0.0};
    double length{0.0};
    bool synthetic{false};
    bool visited{false};
};

struct RawSegment
{
    Point2d start{};
    Point2d end{};
    bool synthetic{false};
};

struct RingCandidate
{
    Polyline2d ring{};
    double area{0.0};
    double perimeter{0.0};
    double syntheticPerimeter{0.0};
    std::size_t branchVertexCount{0};
    std::size_t syntheticBranchVertexCount{0};
    double score{0.0};
};

struct VertexGraph2d
{
    std::vector<Point2d> vertices;
    std::vector<std::size_t> degrees;
};

[[nodiscard]] std::vector<Point2d> SimplifyRingVertices(std::vector<Point2d> points, double eps);

[[nodiscard]] double SideValue(const Point2d& point, const LineSegment2d& line)
{
    return Cross(line.endPoint - line.startPoint, point - line.startPoint);
}

[[nodiscard]] Point2d IntersectInfiniteLines(const Point2d& a1, const Point2d& a2, const Point2d& b1, const Point2d& b2)
{
    const Vector2d r = a2 - a1;
    const Vector2d s = b2 - b1;
    const double denom = Cross(r, s);
    if (std::abs(denom) <= geometry::kDefaultEpsilon)
    {
        return a2;
    }
    const double t = Cross(b1 - a1, s) / denom;
    return a1 + r * t;
}

[[nodiscard]] Polyline2d ClipPolylineLength(const Polyline2d& polyline, double startLength, double endLength)
{
    if (!polyline.IsValid() || endLength < startLength)
    {
        return {};
    }

    const double total = polyline.Length();
    startLength = std::clamp(startLength, 0.0, total);
    endLength = std::clamp(endLength, 0.0, total);
    if (endLength < startLength + geometry::kDefaultEpsilon)
    {
        return {};
    }

    std::vector<Point2d> points;
    double cursor = 0.0;
    for (std::size_t i = 0; i + 1 < polyline.PointCount(); ++i)
    {
        const LineSegment2d segment(polyline.PointAt(i), polyline.PointAt(i + 1));
        const double segLen = segment.Length();
        const double segStart = cursor;
        const double segEnd = cursor + segLen;

        if (segEnd < startLength - geometry::kDefaultEpsilon)
        {
            cursor = segEnd;
            continue;
        }
        if (segStart > endLength + geometry::kDefaultEpsilon)
        {
            break;
        }

        const double localStart = std::max(0.0, startLength - segStart);
        const double localEnd = std::min(segLen, endLength - segStart);
        const Point2d startPoint = segment.PointAtLength(localStart, true);
        const Point2d endPoint = segment.PointAtLength(localEnd, true);
        if (points.empty() || !points.back().AlmostEquals(startPoint, geometry::kDefaultEpsilon))
        {
            points.push_back(startPoint);
        }
        if (points.empty() || !points.back().AlmostEquals(endPoint, geometry::kDefaultEpsilon))
        {
            points.push_back(endPoint);
        }

        cursor = segEnd;
    }

    return Polyline2d(std::move(points), PolylineClosure::Open);
}

[[nodiscard]] Polygon2d ClipPolygonHalfPlane(const Polygon2d& polygon, const LineSegment2d& cutter, bool keepLeft)
{
    if (!polygon.IsValid() || polygon.HoleCount() != 0)
    {
        return {};
    }

    std::vector<Point2d> input;
    input.reserve(polygon.OuterRing().PointCount());
    for (std::size_t i = 0; i < polygon.OuterRing().PointCount(); ++i)
    {
        input.push_back(polygon.OuterRing().PointAt(i));
    }

    std::vector<Point2d> output;
    if (input.empty())
    {
        return {};
    }

    Point2d prev = input.back();
    double prevSide = SideValue(prev, cutter);
    for (const Point2d& current : input)
    {
        const double currentSide = SideValue(current, cutter);
        const bool prevInside = keepLeft ? prevSide >= -geometry::kDefaultEpsilon : prevSide <= geometry::kDefaultEpsilon;
        const bool currentInside =
            keepLeft ? currentSide >= -geometry::kDefaultEpsilon : currentSide <= geometry::kDefaultEpsilon;

        if (currentInside)
        {
            if (!prevInside)
            {
                output.push_back(IntersectInfiniteLines(prev, current, cutter.startPoint, cutter.endPoint));
            }
            output.push_back(current);
        }
        else if (prevInside)
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

    return Polygon2d(Polyline2d(std::move(output), PolylineClosure::Closed));
}

[[nodiscard]] Polygon2d BuildHalfPlaneClipPolygon(
    const Polygon2d& polygon,
    const LineSegment2d& cutter,
    bool keepLeft,
    double eps)
{
    if (!polygon.IsValid())
    {
        return {};
    }

    const Box2d bounds = polygon.Bounds();
    if (!bounds.IsValid())
    {
        return {};
    }

    const Vector2d direction = cutter.endPoint - cutter.startPoint;
    const double length = direction.Length();
    if (length <= eps)
    {
        return {};
    }

    const Vector2d tangent = direction / length;
    Vector2d normal{-tangent.y, tangent.x};
    if (!keepLeft)
    {
        normal = normal * -1.0;
    }

    const Point2d minPoint = bounds.MinPoint();
    const Point2d maxPoint = bounds.MaxPoint();
    const double diagonal = std::sqrt((maxPoint.x - minPoint.x) * (maxPoint.x - minPoint.x) +
                                      (maxPoint.y - minPoint.y) * (maxPoint.y - minPoint.y));
    const double extent = std::max(1.0, 8.0 * diagonal + 1.0);

    const Vector2d along = tangent * extent;
    const Vector2d side = normal * (2.0 * extent);
    const Point2d anchor = cutter.startPoint;
    const Point2d lineStart = anchor - along;
    const Point2d lineEnd = anchor + along;

    Polyline2d ring(
        {
            lineStart,
            lineEnd,
            lineEnd + side,
            lineStart + side,
        },
        PolylineClosure::Closed);
    if (Orientation(ring) != RingOrientation2d::CounterClockwise)
    {
        ring = Reverse(ring);
    }

    const Polygon2d clip(ring);
    if (!clip.IsValid() || !Validate(clip, eps).valid)
    {
        return {};
    }
    return clip;
}

void CollectRawSegments(const Polyline2d& polyline, std::vector<RawSegment>& segments, double eps)
{
    const Polyline2d normalized = Normalize(polyline, eps);
    if (!normalized.IsValid() || normalized.PointCount() < 2)
    {
        return;
    }

    const std::size_t segmentCount = normalized.IsClosed() ? normalized.PointCount() : normalized.PointCount() - 1;
    for (std::size_t i = 0; i < segmentCount; ++i)
    {
        const Point2d start = normalized.PointAt(i);
        const Point2d end = normalized.PointAt((i + 1) % normalized.PointCount());
        if (!start.AlmostEquals(end, eps))
        {
            segments.push_back(RawSegment{start, end, false});
        }
    }
}

void AppendPolygonBoundaries(const Polygon2d& polygon, MultiPolyline2d& boundaries)
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

[[nodiscard]] Polygon2d SelectLargestValidPolygon(const MultiPolygon2d& polygons)
{
    Polygon2d best;
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

[[nodiscard]] Polygon2d RebuildLargestPolygonWithCandidates(const MultiPolyline2d& boundaries, const std::vector<double>& epsCandidates)
{
    for (double candidateEps : epsCandidates)
    {
        const MultiPolygon2d rebuilt = BuildMultiPolygonByLines(boundaries, candidateEps);
        if (rebuilt.Count() == 1 && rebuilt[0].IsValid())
        {
            return rebuilt[0];
        }

        Polygon2d best = SelectLargestValidPolygon(rebuilt);
        if (best.IsValid())
        {
            return best;
        }
    }

    return {};
}

[[nodiscard]] MultiPolyline2d SimplifyBoundaryPolylines(const MultiPolyline2d& boundaries, double eps)
{
    MultiPolyline2d simplifiedBoundaries;
    for (std::size_t boundaryIndex = 0; boundaryIndex < boundaries.Count(); ++boundaryIndex)
    {
        const Polyline2d boundary = boundaries[boundaryIndex];
        std::vector<Point2d> points;
        points.reserve(boundary.PointCount());
        for (std::size_t pointIndex = 0; pointIndex < boundary.PointCount(); ++pointIndex)
        {
            points.push_back(boundary.PointAt(pointIndex));
        }

        std::vector<Point2d> simplifiedPoints = SimplifyRingVertices(std::move(points), eps);
        const bool isClosed = boundary.IsClosed();
        const std::size_t minimumPointCount = isClosed ? 3U : 2U;
        if (simplifiedPoints.size() < minimumPointCount)
        {
            continue;
        }

        Polyline2d simplifiedBoundary(std::move(simplifiedPoints), isClosed ? PolylineClosure::Closed : PolylineClosure::Open);
        if (simplifiedBoundary.IsValid())
        {
            simplifiedBoundaries.Add(std::move(simplifiedBoundary));
        }
    }

    return simplifiedBoundaries;
}

[[nodiscard]] std::vector<RawSegment> CollectRawSegments(const MultiPolyline2d& polylines, double eps)
{
    std::vector<RawSegment> segments;
    for (std::size_t i = 0; i < polylines.Count(); ++i)
    {
        CollectRawSegments(polylines[i], segments, eps);
    }
    return segments;
}

void AddParameter(std::vector<double>& parameters, double value, double eps)
{
    value = std::clamp(value, 0.0, 1.0);
    for (double existing : parameters)
    {
        if (std::abs(existing - value) <= eps)
        {
            return;
        }
    }
    parameters.push_back(value);
}

[[nodiscard]] double ComputeParameterTolerance(const LineSegment2d& segment, double eps)
{
    const double length = segment.Length();
    if (length <= eps)
    {
        return 1e-12;
    }

    return std::min(1e-4, std::max(1e-12, 8.0 * eps / length));
}

[[nodiscard]] std::vector<double> CompactSortedParameters(std::vector<double> parameters, double parameterTol)
{
    if (parameters.empty())
    {
        return parameters;
    }

    std::sort(parameters.begin(), parameters.end());
    std::vector<double> compacted;
    compacted.reserve(parameters.size());

    std::size_t clusterStart = 0;
    while (clusterStart < parameters.size())
    {
        std::size_t clusterEnd = clusterStart + 1;
        double weighted = parameters[clusterStart];
        while (clusterEnd < parameters.size() &&
               parameters[clusterEnd] <= parameters[clusterEnd - 1] + parameterTol)
        {
            weighted += parameters[clusterEnd];
            ++clusterEnd;
        }

        const double representative = weighted / static_cast<double>(clusterEnd - clusterStart);
        compacted.push_back(std::clamp(representative, 0.0, 1.0));
        clusterStart = clusterEnd;
    }

    if (compacted.front() > parameterTol)
    {
        compacted.insert(compacted.begin(), 0.0);
    }
    else
    {
        compacted.front() = 0.0;
    }

    if (compacted.back() < 1.0 - parameterTol)
    {
        compacted.push_back(1.0);
    }
    else
    {
        compacted.back() = 1.0;
    }

    return compacted;
}

[[nodiscard]] std::vector<RawSegment> SubdivideRawSegments(const std::vector<RawSegment>& rawSegments, double eps)
{
    std::vector<std::vector<double>> parameters(rawSegments.size(), std::vector<double>{0.0, 1.0});
    for (std::size_t i = 0; i < rawSegments.size(); ++i)
    {
        const LineSegment2d first(rawSegments[i].start, rawSegments[i].end);
        for (std::size_t j = i + 1; j < rawSegments.size(); ++j)
        {
            const LineSegment2d second(rawSegments[j].start, rawSegments[j].end);
            const SegmentIntersection2d intersection = Intersect(first, second, eps);
            if (!intersection.HasIntersection())
            {
                continue;
            }

            for (std::size_t k = 0; k < intersection.pointCount; ++k)
            {
                AddParameter(parameters[i], intersection.points[k].parameterOnFirst, eps);
                AddParameter(parameters[j], intersection.points[k].parameterOnSecond, eps);
            }
        }
    }

    std::vector<RawSegment> splitSegments;
    for (std::size_t i = 0; i < rawSegments.size(); ++i)
    {
        LineSegment2d segment(rawSegments[i].start, rawSegments[i].end);
        const double parameterTol = ComputeParameterTolerance(segment, eps);
        std::vector<double> params = CompactSortedParameters(parameters[i], parameterTol);

        for (std::size_t k = 0; k + 1 < params.size(); ++k)
        {
            if (params[k + 1] <= params[k] + parameterTol)
            {
                continue;
            }

            Point2d start = segment.PointAt(params[k]);
            Point2d end = segment.PointAt(params[k + 1]);
            if (params[k] <= parameterTol)
            {
                start = segment.startPoint;
            }
            else if (params[k] >= 1.0 - parameterTol)
            {
                start = segment.endPoint;
            }

            if (params[k + 1] <= parameterTol)
            {
                end = segment.startPoint;
            }
            else if (params[k + 1] >= 1.0 - parameterTol)
            {
                end = segment.endPoint;
            }

            if (!start.AlmostEquals(end, eps))
            {
                splitSegments.push_back(RawSegment{start, end, rawSegments[i].synthetic});
            }
        }
    }

    return splitSegments;
}

[[nodiscard]] std::size_t FindVertexIndex(const std::vector<Point2d>& vertices, const Point2d& point, double eps)
{
    for (std::size_t i = 0; i < vertices.size(); ++i)
    {
        if (vertices[i].AlmostEquals(point, eps))
        {
            return i;
        }
    }
    return static_cast<std::size_t>(-1);
}

[[nodiscard]] std::size_t FindOrAddVertex(std::vector<Point2d>& vertices, const Point2d& point, double eps)
{
    const std::size_t existing = FindVertexIndex(vertices, point, eps);
    if (existing != static_cast<std::size_t>(-1))
    {
        return existing;
    }

    vertices.push_back(point);
    return vertices.size() - 1;
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

        const std::size_t from = FindOrAddVertex(graph.vertices, segment.start, eps);
        const std::size_t to = FindOrAddVertex(graph.vertices, segment.end, eps);
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

[[nodiscard]] std::uint64_t MakeUndirectedEdgeKey(std::size_t first, std::size_t second)
{
    const std::uint64_t a = static_cast<std::uint64_t>(std::min(first, second));
    const std::uint64_t b = static_cast<std::uint64_t>(std::max(first, second));
    return (a << 32U) | b;
}

[[nodiscard]] std::vector<RawSegment> RemoveDuplicateSegments(const std::vector<RawSegment>& segments, double eps)
{
    std::vector<RawSegment> unique;
    std::vector<Point2d> vertices;
    std::unordered_map<std::uint64_t, std::size_t> edgeKeys;
    unique.reserve(segments.size());
    for (const RawSegment& segment : segments)
    {
        if (segment.start.AlmostEquals(segment.end, eps))
        {
            continue;
        }

        const std::size_t from = FindOrAddVertex(vertices, segment.start, eps);
        const std::size_t to = FindOrAddVertex(vertices, segment.end, eps);
        if (from == to)
        {
            continue;
        }

        const std::uint64_t key = MakeUndirectedEdgeKey(from, to);
        const auto [it, inserted] = edgeKeys.emplace(key, unique.size());
        if (inserted)
        {
            unique.push_back(segment);
        }
        else if (unique[it->second].synthetic && !segment.synthetic)
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

        const Point2d& first = graph.vertices[endpoints[i]];
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

        const Point2d& endpoint = graph.vertices[vertexIndex];
        SegmentProjection2d bestProjection;
        bool found = false;
        for (std::size_t segmentIndex = 0; segmentIndex < originalCount; ++segmentIndex)
        {
            const RawSegment& segment = segments[segmentIndex];
            if (endpoint.AlmostEquals(segment.start, eps) || endpoint.AlmostEquals(segment.end, eps))
            {
                continue;
            }

            const SegmentProjection2d projection =
                ProjectPointToLineSegment(endpoint, LineSegment2d(segment.start, segment.end), true);
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
            const std::size_t from = FindVertexIndex(graph.vertices, segment.start, eps);
            const std::size_t to = FindVertexIndex(graph.vertices, segment.end, eps);
            if (from == static_cast<std::size_t>(-1) || to == static_cast<std::size_t>(-1) ||
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

void AppendSplitEdges(
    const std::vector<RawSegment>& segments,
    std::vector<Point2d>& vertices,
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

        const std::size_t from = FindOrAddVertex(vertices, segment.start, eps);
        const std::size_t to = FindOrAddVertex(vertices, segment.end, eps);
        if (from == to)
        {
            continue;
        }

        const std::uint64_t key = MakeUndirectedEdgeKey(from, to);
        if (!edgeKeys.insert(key).second)
        {
            continue;
        }

        const Vector2d forward = vertices[to] - vertices[from];
        const Vector2d backward = vertices[from] - vertices[to];

        const std::size_t forwardIndex = edges.size();
        edges.push_back(DirectedEdge{
            from,
            to,
            forwardIndex + 1,
            std::atan2(forward.y, forward.x),
            forward.Length(),
            segment.synthetic,
            false});
        edges.push_back(DirectedEdge{
            to,
            from,
            forwardIndex,
            std::atan2(backward.y, backward.x),
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

void SortOutgoing(const std::vector<DirectedEdge>& edges, std::vector<std::vector<std::size_t>>& outgoing)
{
    for (auto& fan : outgoing)
    {
        std::sort(fan.begin(), fan.end(), [&edges](std::size_t lhs, std::size_t rhs) {
            return edges[lhs].angle < edges[rhs].angle;
        });
    }
}

[[nodiscard]] std::size_t PreviousOutgoing(const std::vector<std::size_t>& fan, std::size_t edgeIndex)
{
    for (std::size_t i = 0; i < fan.size(); ++i)
    {
        if (fan[i] == edgeIndex)
        {
            return fan[(i + fan.size() - 1) % fan.size()];
        }
    }

    return std::numeric_limits<std::size_t>::max();
}

[[nodiscard]] std::size_t NextFaceEdge(
    const std::vector<DirectedEdge>& edges,
    const std::vector<std::vector<std::size_t>>& outgoing,
    std::size_t edgeIndex)
{
    const DirectedEdge& edge = edges[edgeIndex];
    if (edge.to >= outgoing.size())
    {
        return std::numeric_limits<std::size_t>::max();
    }

    const std::vector<std::size_t>& fan = outgoing[edge.to];
    if (fan.empty())
    {
        return std::numeric_limits<std::size_t>::max();
    }

    return PreviousOutgoing(fan, edge.twin);
}

[[nodiscard]] std::vector<Point2d> SimplifyRingVertices(std::vector<Point2d> points, double eps)
{
    std::vector<Point2d> simplified;
    simplified.reserve(points.size());
    for (const Point2d& point : points)
    {
        if (simplified.empty() || !simplified.back().AlmostEquals(point, eps))
        {
            simplified.push_back(point);
        }
    }

    while (simplified.size() >= 2 && simplified.front().AlmostEquals(simplified.back(), eps))
    {
        simplified.pop_back();
    }

    return simplified;
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

[[nodiscard]] bool RejectRingCandidate(const RingCandidate& candidate, double repairTol, double areaTol, double eps)
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
        syntheticRatio >= 0.25 && candidate.area <= candidate.syntheticPerimeter * std::max(repairTol, 16.0 * eps);
    const bool syntheticBranchDominated =
        candidate.syntheticBranchVertexCount > 0 &&
        candidate.syntheticBranchVertexCount * 2U >= candidate.branchVertexCount + 1U &&
        syntheticRatio >= 0.2 &&
        candidate.area <=
            candidate.syntheticPerimeter * std::max(1.5 * repairTol, 24.0 * eps);
    return syntheticDominated || syntheticBranchDominated;
}

[[nodiscard]] double ComputeRingCandidateScore(
    double area,
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
    const std::vector<Point2d>& vertices,
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

        std::vector<Point2d> loopPoints;
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

            const std::size_t next = NextFaceEdge(edges, outgoing, current);
            if (next == std::numeric_limits<std::size_t>::max())
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

        std::vector<Point2d> simplified = SimplifyRingVertices(std::move(loopPoints), eps);
        if (simplified.size() < 3)
        {
            continue;
        }

        Polyline2d ring(std::move(simplified), PolylineClosure::Closed);
        if (!Validate(ring, eps).valid)
        {
            continue;
        }
        if (Orientation(ring) != RingOrientation2d::CounterClockwise)
        {
            continue;
        }

        const double area = std::abs(Polygon2d(ring).Area());
        const double score = ComputeRingCandidateScore(
            area,
            syntheticPerimeter,
            branchVertexCount,
            syntheticBranchVertexCount,
            repairTol,
            eps);
        const RingCandidate candidate{
            ring,
            area,
            perimeter,
            syntheticPerimeter,
            branchVertexCount,
            syntheticBranchVertexCount,
            score};
        if (RejectRingCandidate(candidate, repairTol, areaTol, eps))
        {
            continue;
        }

        rings.push_back(candidate);
    }

    std::stable_sort(rings.begin(), rings.end(), [](const RingCandidate& lhs, const RingCandidate& rhs) {
        if (std::abs(lhs.score - rhs.score) > 1e-12)
        {
            return lhs.score > rhs.score;
        }
        if (std::abs(lhs.area - rhs.area) > 1e-12)
        {
            return lhs.area > rhs.area;
        }
        return lhs.perimeter < rhs.perimeter;
    });

    return rings;
}

[[nodiscard]] std::vector<std::size_t> BuildLoopParents(const std::vector<Polygon2d>& loops, double eps)
{
    std::vector<std::size_t> parents(loops.size(), static_cast<std::size_t>(-1));
    for (std::size_t i = 0; i < loops.size(); ++i)
    {
        const double loopArea = loops[i].Area();
        double bestArea = 0.0;
        for (std::size_t j = 0; j < loops.size(); ++j)
        {
            if (i == j)
            {
                continue;
            }

            const double containerArea = loops[j].Area();
            if (containerArea <= loopArea + eps)
            {
                continue;
            }
            if (!Contains(loops[j], loops[i], eps))
            {
                continue;
            }

            if (parents[i] == static_cast<std::size_t>(-1) || containerArea < bestArea)
            {
                parents[i] = j;
                bestArea = containerArea;
            }
        }
    }

    return parents;
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

[[nodiscard]] MultiPolygon2d BuildFilledPolygonsFromCandidateRings(const std::vector<RingCandidate>& rings, double eps)
{
    MultiPolygon2d result;
    if (rings.empty())
    {
        return result;
    }

    std::vector<Polygon2d> loopPolygons;
    loopPolygons.reserve(rings.size());
    for (const RingCandidate& candidate : rings)
    {
        Polygon2d polygon(candidate.ring);
        if (polygon.IsValid())
        {
            loopPolygons.push_back(std::move(polygon));
        }
    }
    if (loopPolygons.empty())
    {
        return result;
    }

    const std::vector<std::size_t> parents = BuildLoopParents(loopPolygons, eps);
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

        Polyline2d outerRing = loopPolygons[i].OuterRing();
        if (Orientation(outerRing) != RingOrientation2d::CounterClockwise)
        {
            outerRing = Reverse(outerRing);
        }

        std::vector<Polyline2d> holes;
        for (std::size_t j = 0; j < loopPolygons.size(); ++j)
        {
            if (parents[j] != i || (depths[j] % 2U) == 0U)
            {
                continue;
            }

            Polyline2d holeRing = loopPolygons[j].OuterRing();
            if (Orientation(holeRing) != RingOrientation2d::Clockwise)
            {
                holeRing = Reverse(holeRing);
            }
            holes.push_back(std::move(holeRing));
        }

        Polygon2d polygon(outerRing, std::move(holes));
        if (polygon.IsValid() && Validate(polygon, eps).valid)
        {
            result.Add(std::move(polygon));
        }
    }

    return result;
}
} // namespace

Polyline2d SubPolyline(const Polyline2d& polyline, double startLength, double endLength)
{
    return ClipPolylineLength(polyline, startLength, endLength);
}

PolygonCutResult2d CutPolygon(const Polygon2d& polygon, const LineSegment2d& cutter, double eps)
{
    PolygonCutResult2d result;

    if (!polygon.IsValid())
    {
        return result;
    }

    if (polygon.HoleCount() > 0)
    {
        const Polygon2d leftClip = BuildHalfPlaneClipPolygon(polygon, cutter, true, eps);
        const Polygon2d rightClip = BuildHalfPlaneClipPolygon(polygon, cutter, false, eps);
        if (!leftClip.IsValid() || !rightClip.IsValid())
        {
            return result;
        }

        result.left = Intersect(polygon, leftClip, eps);
        result.right = Intersect(polygon, rightClip, eps);
        result.success = !result.left.IsEmpty() && !result.right.IsEmpty();
        return result;
    }

    const Polygon2d left = ClipPolygonHalfPlane(polygon, cutter, true);
    const Polygon2d right = ClipPolygonHalfPlane(polygon, cutter, false);
    if (!left.IsValid() || !right.IsValid())
    {
        return result;
    }
    result.success = true;
    result.left.Add(Polygon2d(left));
    result.right.Add(Polygon2d(right));
    return result;
}

MultiPolygon2d BuildMultiPolygonByLines(const MultiPolyline2d& polylines, double eps)
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

    std::vector<Point2d> vertices;
    std::vector<DirectedEdge> edges;
    std::vector<std::vector<std::size_t>> outgoing;
    std::unordered_set<std::uint64_t> edgeKeys;
    AppendSplitEdges(splitSegments, vertices, edges, outgoing, edgeKeys, eps);
    if (edges.empty())
    {
        return {};
    }

    SortOutgoing(edges, outgoing);
    const std::vector<RingCandidate> rings =
        ExtractCandidateRings(edges, vertices, outgoing, repairTol, areaTol, eps);
    return BuildFilledPolygonsFromCandidateRings(rings, eps);
}

Polygon2d NormalizePolygonByLines(const Polygon2d& polygon, double eps)
{
    MultiPolyline2d boundaries;
    AppendPolygonBoundaries(polygon, boundaries);
    if (boundaries.IsEmpty())
    {
        return {};
    }

    const std::vector<double> epsCandidates{
        std::max(eps, 1e-12),
        std::max(eps, 1e-8),
        std::max(eps, 1e-7)};

    Polygon2d rebuilt = RebuildLargestPolygonWithCandidates(boundaries, epsCandidates);
    if (rebuilt.IsValid())
    {
        return rebuilt;
    }

    const double simplifyEps = std::max(1e-10, 8.0 * std::max(eps, 1e-12));
    MultiPolyline2d simplifiedBoundaries = SimplifyBoundaryPolylines(boundaries, simplifyEps);
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
} // namespace geometry::sdk

