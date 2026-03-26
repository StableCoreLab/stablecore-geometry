#include "sdk/GeometryBoolean.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "algorithm/Predicate2.h"
#include "common/Epsilon.h"
#include "sdk/GeometryIntersection.h"
#include "sdk/GeometryPathOps.h"
#include "sdk/GeometryRelation.h"
#include "sdk/GeometryTopology.h"
#include "sdk/GeometryShapeOps.h"
#include "sdk/GeometryValidation.h"

namespace geometry::sdk
{
namespace
{
enum class BooleanOp
{
    Intersection,
    Union,
    Difference
};

struct DirectedEdge
{
    std::size_t from{0};
    std::size_t to{0};
    std::size_t twin{0};
    double angle{0.0};
    bool visited{false};
};

struct RawSegment
{
    Point2d start{};
    Point2d end{};
};

[[nodiscard]] bool Evaluate(BooleanOp op, bool inFirst, bool inSecond)
{
    switch (op)
    {
    case BooleanOp::Intersection:
        return inFirst && inSecond;
    case BooleanOp::Union:
        return inFirst || inSecond;
    case BooleanOp::Difference:
        return inFirst && !inSecond;
    }
    return false;
}

void CollectRawSegments(const Polyline2d& polyline, std::vector<RawSegment>& segments, double eps)
{
    if (!polyline.IsValid() || polyline.PointCount() < 2)
    {
        return;
    }

    const std::size_t segmentCount = polyline.IsClosed() ? polyline.PointCount() : polyline.PointCount() - 1;
    for (std::size_t i = 0; i < segmentCount; ++i)
    {
        const Point2d start = polyline.PointAt(i);
        const Point2d end = polyline.PointAt((i + 1) % polyline.PointCount());
        if (!start.AlmostEquals(end, eps))
        {
            segments.push_back(RawSegment{start, end});
        }
    }
}

[[nodiscard]] std::vector<RawSegment> CollectBoundarySegments(
    const Polygon2d& first,
    const Polygon2d& second,
    double eps)
{
    std::vector<RawSegment> segments;
    CollectRawSegments(first.OuterRing(), segments, eps);
    for (std::size_t i = 0; i < first.HoleCount(); ++i)
    {
        CollectRawSegments(first.HoleAt(i), segments, eps);
    }

    CollectRawSegments(second.OuterRing(), segments, eps);
    for (std::size_t i = 0; i < second.HoleCount(); ++i)
    {
        CollectRawSegments(second.HoleAt(i), segments, eps);
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
        std::vector<double>& params = parameters[i];
        std::sort(params.begin(), params.end());
        params.erase(
            std::unique(params.begin(), params.end(), [eps](double lhs, double rhs) {
                return std::abs(lhs - rhs) <= eps;
            }),
            params.end());

        for (std::size_t k = 0; k + 1 < params.size(); ++k)
        {
            if (params[k + 1] <= params[k] + eps)
            {
                continue;
            }

            const Point2d start = segment.PointAt(params[k]);
            const Point2d end = segment.PointAt(params[k + 1]);
            if (!start.AlmostEquals(end, eps))
            {
                splitSegments.push_back(RawSegment{start, end});
            }
        }
    }

    return splitSegments;
}

[[nodiscard]] std::size_t FindOrAddVertex(std::vector<Point2d>& vertices, const Point2d& point, double eps)
{
    for (std::size_t i = 0; i < vertices.size(); ++i)
    {
        if (vertices[i].AlmostEquals(point, eps))
        {
            return i;
        }
    }

    vertices.push_back(point);
    return vertices.size() - 1;
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
        if (edgeKeys.emplace(key, unique.size()).second)
        {
            unique.push_back(segment);
        }
    }
    return unique;
}

[[nodiscard]] double ComputeAreaTolerance(const std::vector<RawSegment>& segments, double eps)
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
    return std::max(256.0 * eps * eps, diagonal * std::max(64.0 * eps, diagonal * 1e-6));
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
        edges.push_back(DirectedEdge{from, to, forwardIndex + 1, std::atan2(forward.y, forward.x), false});
        edges.push_back(DirectedEdge{to, from, forwardIndex, std::atan2(backward.y, backward.x), false});

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

[[nodiscard]] std::vector<Polyline2d> ExtractCandidateRings(
    std::vector<DirectedEdge>& edges,
    const std::vector<Point2d>& vertices,
    const std::vector<std::vector<std::size_t>>& outgoing,
    double eps)
{
    std::vector<Polyline2d> rings;
    for (std::size_t start = 0; start < edges.size(); ++start)
    {
        if (edges[start].visited)
        {
            continue;
        }

        std::vector<Point2d> loopPoints;
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

        rings.push_back(std::move(ring));
    }

    return rings;
}

[[nodiscard]] std::vector<std::size_t> BuildLoopParents(const std::vector<Polygon2d>& loops, double eps)
{
    std::vector<std::size_t> parents(loops.size(), static_cast<std::size_t>(-1));
    for (std::size_t i = 0; i < loops.size(); ++i)
    {
        const double loopArea = Area(loops[i]);
        double bestArea = 0.0;
        for (std::size_t j = 0; j < loops.size(); ++j)
        {
            if (i == j)
            {
                continue;
            }

            const double containerArea = Area(loops[j]);
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

[[nodiscard]] std::vector<Polygon2d> BuildBoundedFacesFromCandidateRings(
    const std::vector<Polyline2d>& rings,
    double areaTol,
    double eps)
{
    std::vector<Polygon2d> faces;
    if (rings.empty())
    {
        return faces;
    }

    std::vector<Polygon2d> loopPolygons;
    loopPolygons.reserve(rings.size());
    for (const Polyline2d& ring : rings)
    {
        Polygon2d polygon(ring);
        if (polygon.IsValid())
        {
            loopPolygons.push_back(std::move(polygon));
        }
    }
    if (loopPolygons.empty())
    {
        return faces;
    }

    const std::vector<std::size_t> parents = BuildLoopParents(loopPolygons, eps);
    faces.reserve(loopPolygons.size());
    for (std::size_t i = 0; i < loopPolygons.size(); ++i)
    {
        Polyline2d outerRing = loopPolygons[i].OuterRing();
        if (Orientation(outerRing) != RingOrientation2d::CounterClockwise)
        {
            outerRing = Reverse(outerRing);
        }

        std::vector<Polyline2d> holes;
        for (std::size_t j = 0; j < loopPolygons.size(); ++j)
        {
            if (parents[j] != i)
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
        if (polygon.IsValid() && Validate(polygon, eps).valid && Area(polygon) > areaTol)
        {
            faces.push_back(std::move(polygon));
        }
    }

    return faces;
}

[[nodiscard]] Point2d SampleFacePoint(const Polygon2d& polygon, double eps)
{
    const Point2d centroid = Centroid(polygon);
    if (LocatePoint(centroid, polygon, eps) == PointContainment2d::Inside)
    {
        return centroid;
    }

    const Polyline2d outer = polygon.OuterRing();
    for (std::size_t i = 0; i < outer.PointCount(); ++i)
    {
        const Point2d start = outer.PointAt(i);
        const Point2d end = outer.PointAt((i + 1) % outer.PointCount());
        const Vector2d edge = end - start;
        const double length = edge.Length();
        if (length <= eps)
        {
            continue;
        }

        const Point2d midpoint{0.5 * (start.x + end.x), 0.5 * (start.y + end.y)};
        const Vector2d inward{-edge.y / length, edge.x / length};
        const double step = std::max(32.0 * eps, std::min(0.25 * length, 1e-3));
        const Point2d candidate = midpoint + inward * step;
        if (LocatePoint(candidate, polygon, eps) == PointContainment2d::Inside)
        {
            return candidate;
        }
    }

    return polygon.OuterRing().PointAt(0);
}

void AppendFaceBoundaries(const Polygon2d& polygon, MultiPolyline2d& polylines)
{
    polylines.Add(polygon.OuterRing());
    for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
    {
        polylines.Add(polygon.HoleAt(i));
    }
}

[[nodiscard]] MultiPolygon2d MakeSinglePolygonResult(const Polygon2d& polygon)
{
    MultiPolygon2d result;
    if (polygon.IsValid())
    {
        result.Add(Polygon2d(polygon));
    }
    return result;
}

[[nodiscard]] MultiPolygon2d MakePairResult(const Polygon2d& first, const Polygon2d& second)
{
    MultiPolygon2d result;
    if (first.IsValid())
    {
        result.Add(Polygon2d(first));
    }
    if (second.IsValid())
    {
        result.Add(Polygon2d(second));
    }
    return result;
}

[[nodiscard]] MultiPolygon2d BooleanFastPath(
    const Polygon2d& first,
    const Polygon2d& second,
    PolygonContainment2d relation,
    BooleanOp op)
{
    switch (relation)
    {
    case PolygonContainment2d::Equal:
        if (op == BooleanOp::Difference)
        {
            return {};
        }
        return MakeSinglePolygonResult(first);
    case PolygonContainment2d::Disjoint:
        if (op == BooleanOp::Intersection)
        {
            return {};
        }
        if (op == BooleanOp::Union)
        {
            return MakePairResult(first, second);
        }
        return MakeSinglePolygonResult(first);
    case PolygonContainment2d::FirstContainsSecond:
        if (op == BooleanOp::Intersection)
        {
            return MakeSinglePolygonResult(second);
        }
        if (op == BooleanOp::Union)
        {
            return MakeSinglePolygonResult(first);
        }
        break;
    case PolygonContainment2d::SecondContainsFirst:
        if (op == BooleanOp::Intersection)
        {
            return MakeSinglePolygonResult(first);
        }
        if (op == BooleanOp::Union)
        {
            return MakeSinglePolygonResult(second);
        }
        if (op == BooleanOp::Difference)
        {
            return {};
        }
        break;
    case PolygonContainment2d::Touching:
        if (op == BooleanOp::Intersection)
        {
            return {};
        }
        if (op == BooleanOp::Difference)
        {
            return MakeSinglePolygonResult(first);
        }
        break;
    case PolygonContainment2d::Intersecting:
        break;
    }

    return MultiPolygon2d{};
}

[[nodiscard]] MultiPolygon2d BooleanCompose(
    const Polygon2d& first,
    const Polygon2d& second,
    BooleanOp op,
    double eps)
{
    MultiPolygon2d result;
    if (!first.IsValid() || !second.IsValid())
    {
        return result;
    }

    const PolygonContainment2d relation = Relate(first, second, eps);
    result = BooleanFastPath(first, second, relation, op);
    if (!result.IsEmpty() || relation != PolygonContainment2d::Intersecting)
    {
        return result;
    }

    std::vector<RawSegment> rawSegments = CollectBoundarySegments(first, second, eps);
    if (rawSegments.empty())
    {
        return result;
    }

    rawSegments = RemoveDuplicateSegments(rawSegments, eps);
    const std::vector<RawSegment> splitSegments = SubdivideRawSegments(rawSegments, eps);
    if (splitSegments.empty())
    {
        return result;
    }

    const double areaTol = ComputeAreaTolerance(splitSegments, eps);

    std::vector<Point2d> vertices;
    std::vector<DirectedEdge> edges;
    std::vector<std::vector<std::size_t>> outgoing;
    std::unordered_set<std::uint64_t> edgeKeys;
    AppendSplitEdges(splitSegments, vertices, edges, outgoing, edgeKeys, eps);
    if (edges.empty())
    {
        return result;
    }

    SortOutgoing(edges, outgoing);
    const std::vector<Polyline2d> rings = ExtractCandidateRings(edges, vertices, outgoing, eps);
    const std::vector<Polygon2d> faces = BuildBoundedFacesFromCandidateRings(rings, areaTol, eps);
    if (faces.empty())
    {
        return result;
    }

    MultiPolyline2d selectedBoundaries;
    for (const Polygon2d& face : faces)
    {
        const Point2d sample = SampleFacePoint(face, eps);
        const bool inFirst = LocatePoint(sample, first, eps) == PointContainment2d::Inside;
        const bool inSecond = LocatePoint(sample, second, eps) == PointContainment2d::Inside;
        if (Evaluate(op, inFirst, inSecond))
        {
            AppendFaceBoundaries(face, selectedBoundaries);
        }
    }

    if (selectedBoundaries.IsEmpty())
    {
        return result;
    }

    return BuildMultiPolygonByLines(selectedBoundaries, eps);
}
} // namespace

MultiPolygon2d Intersect(const Polygon2d& first, const Polygon2d& second, double eps)
{
    return BooleanCompose(first, second, BooleanOp::Intersection, eps);
}

MultiPolygon2d Union(const Polygon2d& first, const Polygon2d& second, double eps)
{
    return BooleanCompose(first, second, BooleanOp::Union, eps);
}

MultiPolygon2d Difference(const Polygon2d& first, const Polygon2d& second, double eps)
{
    return BooleanCompose(first, second, BooleanOp::Difference, eps);
}
} // namespace geometry::sdk
