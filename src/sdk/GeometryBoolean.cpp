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
#include "sdk/GeometryEditing.h"
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

struct BooleanFastPathResult
{
    MultiPolygon2d polygons{};
    bool handled{false};
};

struct AxisAlignedBox
{
    double minX{0.0};
    double minY{0.0};
    double maxX{0.0};
    double maxY{0.0};
};

[[nodiscard]] bool IsNearlyCollinearVertex(
    const Point2d& previous,
    const Point2d& current,
    const Point2d& next,
    double eps);

[[nodiscard]] double ComputeVertexMergeTolerance(const std::vector<RawSegment>& segments, double eps);

[[nodiscard]] std::vector<Point2d> SimplifyRingVertices(std::vector<Point2d> points, double eps);

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

[[nodiscard]] Polygon2d MakeAxisAlignedBoxPolygon(const AxisAlignedBox& box)
{
    return Polygon2d(
        Polyline2d(
            {
                Point2d{box.minX, box.minY},
                Point2d{box.maxX, box.minY},
                Point2d{box.maxX, box.maxY},
                Point2d{box.minX, box.maxY},
            },
            PolylineClosure::Closed));
}

[[nodiscard]] bool TryBuildAxisAlignedBox(const Polygon2d& polygon, AxisAlignedBox& box, double eps)
{
    if (!polygon.IsValid() || polygon.HoleCount() != 0 || polygon.OuterRing().PointCount() < 4)
    {
        return false;
    }

    const Polyline2d outer = polygon.OuterRing();
    box.minX = outer.PointAt(0).x;
    box.minY = outer.PointAt(0).y;
    box.maxX = box.minX;
    box.maxY = box.minY;
    for (std::size_t i = 1; i < outer.PointCount(); ++i)
    {
        const Point2d point = outer.PointAt(i);
        box.minX = std::min(box.minX, point.x);
        box.minY = std::min(box.minY, point.y);
        box.maxX = std::max(box.maxX, point.x);
        box.maxY = std::max(box.maxY, point.y);
    }

    const double tol = std::max(1e-12, 0.05 * eps);
    for (std::size_t i = 0; i < outer.PointCount(); ++i)
    {
        const Point2d current = outer.PointAt(i);
        const Point2d next = outer.PointAt((i + 1) % outer.PointCount());
        const double dx = std::abs(next.x - current.x);
        const double dy = std::abs(next.y - current.y);
        if (dx > tol && dy > tol)
        {
            return false;
        }

        const bool onBoundary = std::abs(current.x - box.minX) <= tol || std::abs(current.x - box.maxX) <= tol ||
                                std::abs(current.y - box.minY) <= tol || std::abs(current.y - box.maxY) <= tol;
        if (!onBoundary)
        {
            return false;
        }
    }

    return true;
}

[[nodiscard]] std::vector<Point2d> ClipPointsToAxisAlignedBox(
    const std::vector<Point2d>& points,
    const AxisAlignedBox& box,
    double eps)
{
    auto clipHalfPlane = [eps](
                             const std::vector<Point2d>& input,
                             const auto& inside,
                             const auto& intersection) {
        std::vector<Point2d> output;
        if (input.empty())
        {
            return output;
        }

        Point2d previous = input.back();
        bool previousInside = inside(previous);
        for (const Point2d& current : input)
        {
            const bool currentInside = inside(current);
            if (currentInside)
            {
                if (!previousInside)
                {
                    output.push_back(intersection(previous, current));
                }
                output.push_back(current);
            }
            else if (previousInside)
            {
                output.push_back(intersection(previous, current));
            }

            previous = current;
            previousInside = currentInside;
        }

        std::vector<Point2d> deduped;
        deduped.reserve(output.size());
        for (const Point2d& point : output)
        {
            if (deduped.empty() || !deduped.back().AlmostEquals(point, eps))
            {
                deduped.push_back(point);
            }
        }
        while (deduped.size() >= 2 && deduped.front().AlmostEquals(deduped.back(), eps))
        {
            deduped.pop_back();
        }

        return deduped;
    };

    std::vector<Point2d> output = points;
    output = clipHalfPlane(
        output,
        [&box, eps](const Point2d& p) { return p.x >= box.minX - eps; },
        [&box](const Point2d& a, const Point2d& b) {
            const double dx = b.x - a.x;
            const double t = std::abs(dx) <= 1e-15 ? 0.0 : (box.minX - a.x) / dx;
            return Point2d{box.minX, a.y + t * (b.y - a.y)};
        });
    output = clipHalfPlane(
        output,
        [&box, eps](const Point2d& p) { return p.x <= box.maxX + eps; },
        [&box](const Point2d& a, const Point2d& b) {
            const double dx = b.x - a.x;
            const double t = std::abs(dx) <= 1e-15 ? 0.0 : (box.maxX - a.x) / dx;
            return Point2d{box.maxX, a.y + t * (b.y - a.y)};
        });
    output = clipHalfPlane(
        output,
        [&box, eps](const Point2d& p) { return p.y >= box.minY - eps; },
        [&box](const Point2d& a, const Point2d& b) {
            const double dy = b.y - a.y;
            const double t = std::abs(dy) <= 1e-15 ? 0.0 : (box.minY - a.y) / dy;
            return Point2d{a.x + t * (b.x - a.x), box.minY};
        });
    output = clipHalfPlane(
        output,
        [&box, eps](const Point2d& p) { return p.y <= box.maxY + eps; },
        [&box](const Point2d& a, const Point2d& b) {
            const double dy = b.y - a.y;
            const double t = std::abs(dy) <= 1e-15 ? 0.0 : (box.maxY - a.y) / dy;
            return Point2d{a.x + t * (b.x - a.x), box.maxY};
        });

    return output;
}

[[nodiscard]] MultiPolygon2d TryAxisAlignedBoxIntersectionFallback(
    const Polygon2d& subject,
    const Polygon2d& clipper,
    double eps)
{
    MultiPolygon2d fallback;
    if (subject.HoleCount() != 0)
    {
        return fallback;
    }

    AxisAlignedBox box;
    if (!TryBuildAxisAlignedBox(clipper, box, eps))
    {
        return fallback;
    }

    const Polyline2d normalizedOuter = Normalize(subject.OuterRing(), std::max(eps, 1e-8));
    if (!normalizedOuter.IsValid() || normalizedOuter.PointCount() < 3)
    {
        return fallback;
    }

    std::vector<Point2d> subjectPoints;
    const Polyline2d outer = normalizedOuter;
    subjectPoints.reserve(outer.PointCount());
    for (std::size_t i = 0; i < outer.PointCount(); ++i)
    {
        subjectPoints.push_back(outer.PointAt(i));
    }

    std::vector<Point2d> clipped = ClipPointsToAxisAlignedBox(subjectPoints, box, eps);
    clipped = SimplifyRingVertices(std::move(clipped), std::max(eps, 1e-8));
    if (clipped.size() < 3)
    {
        return fallback;
    }

    Polyline2d clippedRing(std::move(clipped), PolylineClosure::Closed);
    if (Orientation(clippedRing) != RingOrientation2d::CounterClockwise)
    {
        clippedRing = Reverse(clippedRing);
    }

    Polygon2d clippedPolygon(std::move(clippedRing));
    if (!clippedPolygon.IsValid() || !Validate(clippedPolygon, eps).valid)
    {
        MultiPolyline2d boundaries;
        boundaries.Add(clippedPolygon.OuterRing());
        const MultiPolygon2d rebuilt = BuildMultiPolygonByLines(boundaries, std::max(eps, 1e-8));
        Polygon2d best;
        double bestArea = 0.0;
        for (std::size_t i = 0; i < rebuilt.Count(); ++i)
        {
            if (!rebuilt[i].IsValid())
            {
                continue;
            }

            const double candidateArea = std::abs(Area(rebuilt[i]));
            if (candidateArea > bestArea)
            {
                best = rebuilt[i];
                bestArea = candidateArea;
            }
        }

        if (!best.IsValid())
        {
            return fallback;
        }
        clippedPolygon = best;
    }
    if (std::abs(Area(clippedPolygon)) <= std::max(64.0 * eps * eps, 1e-14))
    {
        return fallback;
    }

    fallback.Add(std::move(clippedPolygon));
    return fallback;
}

[[nodiscard]] MultiPolygon2d TryAxisAlignedBoxDifferenceFallback(
    const Polygon2d& subject,
    const Polygon2d& clipper,
    double eps)
{
    MultiPolygon2d fallback;
    if (subject.HoleCount() != 0 || subject.OuterRing().PointCount() != 4)
    {
        return fallback;
    }

    AxisAlignedBox subjectBox;
    if (!TryBuildAxisAlignedBox(subject, subjectBox, eps))
    {
        return fallback;
    }

    const MultiPolygon2d overlap = TryAxisAlignedBoxIntersectionFallback(clipper, subject, eps);
    if (overlap.IsEmpty())
    {
        fallback.Add(MakeAxisAlignedBoxPolygon(subjectBox));
        return fallback;
    }
    if (overlap.Count() != 1)
    {
        return {};
    }

    AxisAlignedBox overlapBox;
    if (!TryBuildAxisAlignedBox(overlap[0], overlapBox, eps))
    {
        return {};
    }

    const double areaTol = std::max(1e-10, 64.0 * eps * eps);
    auto addStrip = [&](double minX, double minY, double maxX, double maxY) {
        if (maxX - minX <= eps || maxY - minY <= eps)
        {
            return;
        }

        AxisAlignedBox strip{minX, minY, maxX, maxY};
        Polygon2d polygon = MakeAxisAlignedBoxPolygon(strip);
        if (polygon.IsValid() && std::abs(Area(polygon)) > areaTol)
        {
            fallback.Add(std::move(polygon));
        }
    };

    addStrip(subjectBox.minX, subjectBox.minY, overlapBox.minX, subjectBox.maxY);
    addStrip(overlapBox.maxX, subjectBox.minY, subjectBox.maxX, subjectBox.maxY);
    addStrip(overlapBox.minX, subjectBox.minY, overlapBox.maxX, overlapBox.minY);
    addStrip(overlapBox.minX, overlapBox.maxY, overlapBox.maxX, subjectBox.maxY);
    return fallback;
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

void AppendPolygonBoundaries(const Polygon2d& polygon, MultiPolyline2d& boundaries)
{
    boundaries.Add(polygon.OuterRing());
    for (std::size_t holeIndex = 0; holeIndex < polygon.HoleCount(); ++holeIndex)
    {
        boundaries.Add(polygon.HoleAt(holeIndex));
    }
}

void AppendMultiPolygonBoundaries(const MultiPolygon2d& polygons, MultiPolyline2d& boundaries)
{
    for (std::size_t polygonIndex = 0; polygonIndex < polygons.Count(); ++polygonIndex)
    {
        AppendPolygonBoundaries(polygons[polygonIndex], boundaries);
    }
}

void AppendPolygons(const MultiPolygon2d& source, MultiPolygon2d& destination)
{
    for (std::size_t polygonIndex = 0; polygonIndex < source.Count(); ++polygonIndex)
    {
        destination.Add(Polygon2d(source[polygonIndex]));
    }
}

[[nodiscard]] double TotalArea(const MultiPolygon2d& polygons)
{
    double total = 0.0;
    for (std::size_t polygonIndex = 0; polygonIndex < polygons.Count(); ++polygonIndex)
    {
        total += std::abs(Area(polygons[polygonIndex]));
    }
    return total;
}

[[nodiscard]] Polygon2d NormalizeBooleanOperand(const Polygon2d& polygon, double eps)
{
    const bool inputValid = polygon.IsValid();
    MultiPolyline2d boundaries;
    AppendPolygonBoundaries(polygon, boundaries);
    if (!boundaries.IsEmpty())
    {
        const MultiPolygon2d rebuilt = BuildMultiPolygonByLines(boundaries, eps);
        if (rebuilt.Count() == 1 && rebuilt[0].IsValid())
        {
            return rebuilt[0];
        }

        if (!inputValid)
        {
            // Retry with a slightly relaxed tolerance and keep the largest valid
            // component. Near-duplicate edge families often split into tiny loops
            // at strict eps and recover under this pass.
            const MultiPolygon2d rebuiltRelaxed = BuildMultiPolygonByLines(boundaries, std::max(eps, 1e-8));
            Polygon2d best;
            double bestArea = 0.0;
            for (std::size_t i = 0; i < rebuiltRelaxed.Count(); ++i)
            {
                if (!rebuiltRelaxed[i].IsValid())
                {
                    continue;
                }

                const double candidateArea = std::abs(Area(rebuiltRelaxed[i]));
                if (candidateArea > bestArea)
                {
                    best = rebuiltRelaxed[i];
                    bestArea = candidateArea;
                }
            }
            if (best.IsValid())
            {
                return best;
            }
        }
    }

    if (inputValid)
    {
        return polygon;
    }

    // Near-degenerate duplicate-edge families can fail direct rebuild; simplify
    // ring vertices and retry with a conservative tolerance before giving up.
    const Polyline2d outer = polygon.OuterRing();
    std::vector<Point2d> outerPoints;
    outerPoints.reserve(outer.PointCount());
    for (std::size_t i = 0; i < outer.PointCount(); ++i)
    {
        outerPoints.push_back(outer.PointAt(i));
    }

    const double simplifyEps = std::max(1e-12, 8.0 * eps);
    std::vector<Point2d> simplifiedOuter = SimplifyRingVertices(std::move(outerPoints), simplifyEps);
    if (simplifiedOuter.size() >= 3)
    {
        Polyline2d simplifiedRing(std::move(simplifiedOuter), PolylineClosure::Closed);
        if (Orientation(simplifiedRing) != RingOrientation2d::CounterClockwise)
        {
            simplifiedRing = Reverse(simplifiedRing);
        }

        Polygon2d simplifiedPolygon(std::move(simplifiedRing));
        if (simplifiedPolygon.IsValid() && Validate(simplifiedPolygon, eps).valid)
        {
            return simplifiedPolygon;
        }
    }

    return {};
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
    const double vertexTol = ComputeVertexMergeTolerance(segments, eps);
    unique.reserve(segments.size());
    for (const RawSegment& segment : segments)
    {
        if (segment.start.AlmostEquals(segment.end, eps))
        {
            continue;
        }

        const std::size_t from = FindOrAddVertex(vertices, segment.start, vertexTol);
        const std::size_t to = FindOrAddVertex(vertices, segment.end, vertexTol);
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

[[nodiscard]] double ComputeSegmentLengthTolerance(const std::vector<RawSegment>& segments, double eps)
{
    (void)segments;
    // Only remove clearly negligible fragments. Using a larger threshold can
    // break ring connectivity when near-duplicate vertices encode valid edges.
    return std::max(1e-12, 0.25 * eps);
}

[[nodiscard]] double ComputeVertexMergeTolerance(const std::vector<RawSegment>& segments, double eps)
{
    double minLength = std::numeric_limits<double>::infinity();
    for (const RawSegment& segment : segments)
    {
        const double length = (segment.end - segment.start).Length();
        if (length > 0.0)
        {
            minLength = std::min(minLength, length);
        }
    }

    if (!std::isfinite(minLength))
    {
        return eps;
    }

    const double floorTol = std::max(1e-12, eps * 1e-3);
    return std::max(floorTol, std::min(eps, 0.25 * minLength));
}

[[nodiscard]] std::vector<RawSegment> FilterTinySegments(const std::vector<RawSegment>& segments, double eps)
{
    const double lengthTol = ComputeSegmentLengthTolerance(segments, eps);
    std::vector<RawSegment> filtered;
    filtered.reserve(segments.size());
    for (const RawSegment& segment : segments)
    {
        if ((segment.end - segment.start).Length() <= lengthTol)
        {
            continue;
        }
        filtered.push_back(segment);
    }
    return filtered;
}

[[nodiscard]] std::vector<RawSegment> MergeCollinearChainSegments(const std::vector<RawSegment>& segments, double eps)
{
    std::vector<RawSegment> working = segments;
    bool changed = true;
    while (changed)
    {
        changed = false;

        std::vector<Point2d> vertices;
        std::vector<std::size_t> degree;
        std::vector<std::vector<std::size_t>> incident;
        struct IndexedSegment
        {
            std::size_t from{0};
            std::size_t to{0};
        };
        std::vector<IndexedSegment> indexed;
        indexed.reserve(working.size());
        const double vertexTol = ComputeVertexMergeTolerance(working, eps);

        for (const RawSegment& segment : working)
        {
            const std::size_t from = FindOrAddVertex(vertices, segment.start, vertexTol);
            const std::size_t to = FindOrAddVertex(vertices, segment.end, vertexTol);
            if (degree.size() < vertices.size())
            {
                degree.resize(vertices.size(), 0);
                incident.resize(vertices.size());
            }

            const std::size_t segmentIndex = indexed.size();
            indexed.push_back(IndexedSegment{from, to});
            if (from != to)
            {
                ++degree[from];
                ++degree[to];
                incident[from].push_back(segmentIndex);
                incident[to].push_back(segmentIndex);
            }
        }

        for (std::size_t vertexIndex = 0; vertexIndex < vertices.size(); ++vertexIndex)
        {
            if (vertexIndex >= degree.size() || degree[vertexIndex] != 2 || incident[vertexIndex].size() != 2)
            {
                continue;
            }

            const std::size_t firstIndex = incident[vertexIndex][0];
            const std::size_t secondIndex = incident[vertexIndex][1];
            if (firstIndex >= indexed.size() || secondIndex >= indexed.size() || firstIndex == secondIndex)
            {
                continue;
            }

            const IndexedSegment& first = indexed[firstIndex];
            const IndexedSegment& second = indexed[secondIndex];
            const std::size_t firstOther = first.from == vertexIndex ? first.to : first.from;
            const std::size_t secondOther = second.from == vertexIndex ? second.to : second.from;
            if (firstOther == secondOther)
            {
                continue;
            }

            if (!IsNearlyCollinearVertex(vertices[firstOther], vertices[vertexIndex], vertices[secondOther], eps))
            {
                continue;
            }

            std::vector<RawSegment> merged;
            merged.reserve(working.size() - 1);
            for (std::size_t i = 0; i < working.size(); ++i)
            {
                if (i == firstIndex || i == secondIndex)
                {
                    continue;
                }
                merged.push_back(working[i]);
            }

            const Point2d mergedStart = vertices[firstOther];
            const Point2d mergedEnd = vertices[secondOther];
            if (!mergedStart.AlmostEquals(mergedEnd, eps))
            {
                merged.push_back(RawSegment{mergedStart, mergedEnd});
            }

            working = RemoveDuplicateSegments(merged, eps);
            changed = true;
            break;
        }
    }

    return working;
}

[[nodiscard]] double ComputeAreaTolerance(const std::vector<RawSegment>& segments, double eps)
{
    if (segments.empty())
    {
        return 64.0 * eps * eps;
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
    // Keep the tiny-face filter conservative enough to suppress numerical slivers,
    // but not so aggressive that ultra-thin repeated-overlap bands are erased.
    return std::max(64.0 * eps * eps, 2.0 * diagonal * eps);
}

[[nodiscard]] bool IsNearlyCollinearVertex(
    const Point2d& previous,
    const Point2d& current,
    const Point2d& next,
    double eps)
{
    const Vector2d incoming = current - previous;
    const Vector2d outgoing = next - current;
    const double incomingLength = incoming.Length();
    const double outgoingLength = outgoing.Length();
    if (incomingLength <= eps || outgoingLength <= eps)
    {
        return true;
    }

    const double crossMagnitude = std::abs(Cross(incoming, outgoing));
    if (crossMagnitude > eps * (incomingLength + outgoingLength))
    {
        return false;
    }

    return LocatePoint(current, LineSegment2d(previous, next), eps) == PointContainment2d::OnBoundary;
}

[[nodiscard]] double ComputePolygonScale(const Polygon2d& polygon)
{
    const Polyline2d outer = polygon.OuterRing();
    if (outer.PointCount() == 0)
    {
        return 0.0;
    }

    double minX = outer.PointAt(0).x;
    double minY = outer.PointAt(0).y;
    double maxX = minX;
    double maxY = minY;
    for (std::size_t i = 1; i < outer.PointCount(); ++i)
    {
        const Point2d point = outer.PointAt(i);
        minX = std::min(minX, point.x);
        minY = std::min(minY, point.y);
        maxX = std::max(maxX, point.x);
        maxY = std::max(maxY, point.y);
    }

    const double dx = maxX - minX;
    const double dy = maxY - minY;
    return std::sqrt(dx * dx + dy * dy);
}

struct PolygonBounds
{
    double minX{0.0};
    double minY{0.0};
    double maxX{0.0};
    double maxY{0.0};
};

[[nodiscard]] PolygonBounds ComputePolygonBounds(const Polygon2d& polygon)
{
    const Polyline2d outer = polygon.OuterRing();
    PolygonBounds bounds{};
    if (outer.PointCount() == 0)
    {
        return bounds;
    }

    bounds.minX = outer.PointAt(0).x;
    bounds.minY = outer.PointAt(0).y;
    bounds.maxX = bounds.minX;
    bounds.maxY = bounds.minY;
    for (std::size_t i = 1; i < outer.PointCount(); ++i)
    {
        const Point2d point = outer.PointAt(i);
        bounds.minX = std::min(bounds.minX, point.x);
        bounds.minY = std::min(bounds.minY, point.y);
        bounds.maxX = std::max(bounds.maxX, point.x);
        bounds.maxY = std::max(bounds.maxY, point.y);
    }

    return bounds;
}

[[nodiscard]] bool BoundsOverlap(const Polygon2d& first, const Polygon2d& second, double eps)
{
    const PolygonBounds a = ComputePolygonBounds(first);
    const PolygonBounds b = ComputePolygonBounds(second);
    if (a.maxX < b.minX - eps || b.maxX < a.minX - eps)
    {
        return false;
    }
    if (a.maxY < b.minY - eps || b.maxY < a.minY - eps)
    {
        return false;
    }
    return true;
}

[[nodiscard]] bool BoundsContain(const Polygon2d& outer, const Polygon2d& inner, double eps)
{
    const PolygonBounds a = ComputePolygonBounds(outer);
    const PolygonBounds b = ComputePolygonBounds(inner);
    return a.minX <= b.minX + eps && a.minY <= b.minY + eps && a.maxX + eps >= b.maxX &&
           a.maxY + eps >= b.maxY;
}

[[nodiscard]] PointContainment2d ClassifyFaceAgainstPolygon(
    const Polygon2d& face,
    const Point2d& sample,
    const Polygon2d& polygon,
    double eps)
{
    int insideCount = 0;
    int outsideCount = 0;
    auto accumulate = [&](const Point2d& candidate) {
        if (LocatePoint(candidate, face, eps) != PointContainment2d::Inside)
        {
            return;
        }

        const PointContainment2d containment = LocatePoint(candidate, polygon, eps);
        if (containment == PointContainment2d::Inside)
        {
            ++insideCount;
        }
        else if (containment == PointContainment2d::Outside)
        {
            ++outsideCount;
        }
    };

    accumulate(Centroid(face));
    accumulate(sample);

    const double scale = ComputePolygonScale(face);
    const double baseStep = std::max(64.0 * eps, std::min(0.05 * scale, 1e-3));
    const double steps[] = {baseStep, 0.5 * baseStep, 2.0 * baseStep};
    const Vector2d offsets[] = {
        {1.0, 0.0},
        {-1.0, 0.0},
        {0.0, 1.0},
        {0.0, -1.0},
        {0.7071067811865476, 0.7071067811865476},
        {0.7071067811865476, -0.7071067811865476},
        {-0.7071067811865476, 0.7071067811865476},
        {-0.7071067811865476, -0.7071067811865476}};
    for (double step : steps)
    {
        for (const Vector2d& offset : offsets)
        {
            accumulate(sample + offset * step);
        }
    }

    if (insideCount > 0 && outsideCount == 0)
    {
        return PointContainment2d::Inside;
    }
    if (outsideCount > 0 && insideCount == 0)
    {
        return PointContainment2d::Outside;
    }
    if (insideCount > outsideCount)
    {
        return PointContainment2d::Inside;
    }
    if (outsideCount > insideCount)
    {
        return PointContainment2d::Outside;
    }

    return LocatePoint(sample, polygon, eps);
}

void AppendSplitEdges(
    const std::vector<RawSegment>& segments,
    std::vector<Point2d>& vertices,
    std::vector<DirectedEdge>& edges,
    std::vector<std::vector<std::size_t>>& outgoing,
    std::unordered_set<std::uint64_t>& edgeKeys,
    double eps,
    double vertexTol)
{
    for (const RawSegment& segment : segments)
    {
        if (segment.start.AlmostEquals(segment.end, eps))
        {
            continue;
        }

        const std::size_t from = FindOrAddVertex(vertices, segment.start, vertexTol);
        const std::size_t to = FindOrAddVertex(vertices, segment.end, vertexTol);
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

    bool changed = true;
    while (changed && simplified.size() >= 3)
    {
        changed = false;
        for (std::size_t i = 0; i < simplified.size(); ++i)
        {
            const std::size_t previousIndex = (i + simplified.size() - 1) % simplified.size();
            const std::size_t nextIndex = (i + 1) % simplified.size();
            if (!IsNearlyCollinearVertex(simplified[previousIndex], simplified[i], simplified[nextIndex], eps))
            {
                continue;
            }

            simplified.erase(simplified.begin() + static_cast<std::ptrdiff_t>(i));
            changed = true;
            break;
        }
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
    double minX = outer.PointAt(0).x;
    double minY = outer.PointAt(0).y;
    double maxX = minX;
    double maxY = minY;
    for (std::size_t i = 1; i < outer.PointCount(); ++i)
    {
        const Point2d point = outer.PointAt(i);
        minX = std::min(minX, point.x);
        minY = std::min(minY, point.y);
        maxX = std::max(maxX, point.x);
        maxY = std::max(maxY, point.y);
    }
    const double dx = maxX - minX;
    const double dy = maxY - minY;
    const double polygonScale = std::sqrt(dx * dx + dy * dy);
    const double maxStep = std::max(128.0 * eps, 0.1 * polygonScale);
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
        const double step = std::max(32.0 * eps, std::min(0.25 * length, maxStep));
        const Point2d candidate = midpoint + inward * step;
        if (LocatePoint(candidate, polygon, eps) == PointContainment2d::Inside)
        {
            return candidate;
        }
    }

    for (std::size_t i = 0; i < outer.PointCount(); ++i)
    {
        const Point2d prev = outer.PointAt((i + outer.PointCount() - 1) % outer.PointCount());
        const Point2d current = outer.PointAt(i);
        const Point2d next = outer.PointAt((i + 1) % outer.PointCount());
        const Vector2d incoming = current - prev;
        const Vector2d outgoing = next - current;
        const double incomingLength = incoming.Length();
        const double outgoingLength = outgoing.Length();
        if (incomingLength <= eps || outgoingLength <= eps)
        {
            continue;
        }

        if (Cross(incoming, outgoing) <= eps)
        {
            continue;
        }

        const Vector2d incomingNormal{-incoming.y / incomingLength, incoming.x / incomingLength};
        const Vector2d outgoingNormal{-outgoing.y / outgoingLength, outgoing.x / outgoingLength};
        Vector2d bisector = incomingNormal + outgoingNormal;
        const double bisectorLength = bisector.Length();
        if (bisectorLength <= eps)
        {
            continue;
        }

        bisector = bisector / bisectorLength;
        const double localScale = std::min(incomingLength, outgoingLength);
        const double step = std::max(32.0 * eps, std::min(0.2 * localScale, maxStep));
        const Point2d candidate = current + bisector * step;
        if (LocatePoint(candidate, polygon, eps) == PointContainment2d::Inside)
        {
            return candidate;
        }

        const Point2d triangleCentroid{
            (prev.x + current.x + next.x) / 3.0,
            (prev.y + current.y + next.y) / 3.0};
        if (LocatePoint(triangleCentroid, polygon, eps) == PointContainment2d::Inside)
        {
            return triangleCentroid;
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

[[nodiscard]] BooleanFastPathResult BooleanFastPath(
    const Polygon2d& first,
    const Polygon2d& second,
    PolygonContainment2d relation,
    BooleanOp op,
    double eps)
{
    switch (relation)
    {
    case PolygonContainment2d::Equal:
    {
        if (op == BooleanOp::Difference)
        {
            return {MultiPolygon2d{}, true};
        }
        return {MakeSinglePolygonResult(first), true};
    }
    case PolygonContainment2d::Disjoint:
    {
        if (BoundsOverlap(first, second, eps))
        {
            // Relate() can classify near-degenerate overlap families as disjoint
            // under tolerance pressure; fall back to arrangement solve.
            break;
        }
        if (op == BooleanOp::Intersection)
        {
            return {MultiPolygon2d{}, true};
        }
        if (op == BooleanOp::Union)
        {
            return {MakePairResult(first, second), true};
        }
        return {MakeSinglePolygonResult(first), true};
    }
    case PolygonContainment2d::FirstContainsSecond:
    {
        if (!BoundsContain(first, second, eps))
        {
            break;
        }
        if (op == BooleanOp::Intersection)
        {
            return {MakeSinglePolygonResult(second), true};
        }
        if (op == BooleanOp::Union)
        {
            return {MakeSinglePolygonResult(first), true};
        }
        break;
    }
    case PolygonContainment2d::SecondContainsFirst:
    {
        if (!BoundsContain(second, first, eps))
        {
            break;
        }
        if (op == BooleanOp::Intersection)
        {
            return {MakeSinglePolygonResult(first), true};
        }
        if (op == BooleanOp::Union)
        {
            return {MakeSinglePolygonResult(second), true};
        }
        if (op == BooleanOp::Difference)
        {
            return {MultiPolygon2d{}, true};
        }
        break;
    }
    case PolygonContainment2d::Touching:
    {
        // Touching can hide ultra-thin but non-zero overlaps under tight tolerances.
        // Let arrangement classification decide instead of short-circuiting.
        break;
    }
    case PolygonContainment2d::Intersecting:
        break;
    }

    return {};
}

[[nodiscard]] MultiPolygon2d BooleanCompose(
    const Polygon2d& first,
    const Polygon2d& second,
    BooleanOp op,
    double eps)
{
    MultiPolygon2d result;
    const Polygon2d normalizedFirst = NormalizeBooleanOperand(first, eps);
    const Polygon2d normalizedSecond = NormalizeBooleanOperand(second, eps);
    if (!normalizedFirst.IsValid() || !normalizedSecond.IsValid())
    {
        if (op == BooleanOp::Intersection)
        {
            MultiPolygon2d clipped = TryAxisAlignedBoxIntersectionFallback(first, second, eps);
            if (!clipped.IsEmpty())
            {
                return clipped;
            }

            clipped = TryAxisAlignedBoxIntersectionFallback(second, first, eps);
            if (!clipped.IsEmpty())
            {
                return clipped;
            }
        }
        return result;
    }

    const PolygonContainment2d relation = Relate(normalizedFirst, normalizedSecond, eps);
    const BooleanFastPathResult fastPath = BooleanFastPath(normalizedFirst, normalizedSecond, relation, op, eps);
    if (fastPath.handled)
    {
        return fastPath.polygons;
    }

    std::vector<RawSegment> rawSegments = CollectBoundarySegments(normalizedFirst, normalizedSecond, eps);
    if (rawSegments.empty())
    {
        return result;
    }

    rawSegments = RemoveDuplicateSegments(rawSegments, eps);
    rawSegments = FilterTinySegments(rawSegments, eps);
    std::vector<RawSegment> splitSegments = SubdivideRawSegments(rawSegments, eps);
    splitSegments = FilterTinySegments(splitSegments, eps);
    splitSegments = MergeCollinearChainSegments(splitSegments, eps);
    splitSegments = RemoveDuplicateSegments(splitSegments, eps);
    if (splitSegments.empty())
    {
        return result;
    }

    const double areaTol = ComputeAreaTolerance(splitSegments, eps);

    std::vector<Point2d> vertices;
    std::vector<DirectedEdge> edges;
    std::vector<std::vector<std::size_t>> outgoing;
    std::unordered_set<std::uint64_t> edgeKeys;
    const double splitVertexTol = ComputeVertexMergeTolerance(splitSegments, eps);
    AppendSplitEdges(splitSegments, vertices, edges, outgoing, edgeKeys, eps, splitVertexTol);
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
        const PointContainment2d inFirstClass = ClassifyFaceAgainstPolygon(face, sample, normalizedFirst, eps);
        const PointContainment2d inSecondClass = ClassifyFaceAgainstPolygon(face, sample, normalizedSecond, eps);
        const bool inFirst = inFirstClass == PointContainment2d::Inside;
        const bool inSecond = inSecondClass == PointContainment2d::Inside;
        if (Evaluate(op, inFirst, inSecond))
        {
            AppendFaceBoundaries(face, selectedBoundaries);
        }
    }

    if (selectedBoundaries.IsEmpty() && op == BooleanOp::Intersection && BoundsOverlap(normalizedFirst, normalizedSecond, eps))
    {
        for (const Polygon2d& face : faces)
        {
            const Point2d sample = SampleFacePoint(face, eps);
            const PointContainment2d inFirstClass = ClassifyFaceAgainstPolygon(face, sample, normalizedFirst, eps);
            const PointContainment2d inSecondClass = ClassifyFaceAgainstPolygon(face, sample, normalizedSecond, eps);
            const bool inFirst = inFirstClass != PointContainment2d::Outside;
            const bool inSecond = inSecondClass != PointContainment2d::Outside;
            if (inFirst && inSecond)
            {
                AppendFaceBoundaries(face, selectedBoundaries);
            }
        }
    }

    if (selectedBoundaries.IsEmpty())
    {
        if (op == BooleanOp::Intersection && BoundsOverlap(normalizedFirst, normalizedSecond, eps))
        {
            MultiPolygon2d clipped = TryAxisAlignedBoxIntersectionFallback(normalizedFirst, normalizedSecond, eps);
            if (!clipped.IsEmpty())
            {
                return clipped;
            }

            clipped = TryAxisAlignedBoxIntersectionFallback(normalizedSecond, normalizedFirst, eps);
            if (!clipped.IsEmpty())
            {
                return clipped;
            }
        }
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
    MultiPolygon2d result = BooleanCompose(first, second, BooleanOp::Union, eps);

    const MultiPolygon2d overlap = BooleanCompose(first, second, BooleanOp::Intersection, eps);
    const MultiPolygon2d firstOnly = BooleanCompose(first, second, BooleanOp::Difference, eps);
    MultiPolygon2d secondOnly = BooleanCompose(second, first, BooleanOp::Difference, eps);
    if (secondOnly.IsEmpty() && BoundsOverlap(first, second, eps))
    {
        secondOnly = TryAxisAlignedBoxDifferenceFallback(second, first, eps);
    }
    const double expectedArea = TotalArea(overlap) + TotalArea(firstOnly) + TotalArea(secondOnly);
    const double actualArea = TotalArea(result);
    const double areaTol = std::max(1e-10, 64.0 * eps * eps);

    if (expectedArea > areaTol && actualArea + areaTol < expectedArea)
    {
        MultiPolyline2d boundaries;
        AppendMultiPolygonBoundaries(overlap, boundaries);
        AppendMultiPolygonBoundaries(firstOnly, boundaries);
        AppendMultiPolygonBoundaries(secondOnly, boundaries);
        if (!boundaries.IsEmpty())
        {
            MultiPolygon2d rebuilt = BuildMultiPolygonByLines(boundaries, std::max(eps, 1e-8));
            if (!rebuilt.IsEmpty() && TotalArea(rebuilt) + areaTol >= expectedArea)
            {
                result = std::move(rebuilt);
            }
        }

        if (TotalArea(result) + areaTol < expectedArea)
        {
            MultiPolygon2d decomposed;
            AppendPolygons(firstOnly, decomposed);
            AppendPolygons(secondOnly, decomposed);
            AppendPolygons(overlap, decomposed);
            if (!decomposed.IsEmpty() && TotalArea(decomposed) + areaTol >= expectedArea)
            {
                result = std::move(decomposed);
            }
        }
    }

    return result;
}

MultiPolygon2d Difference(const Polygon2d& first, const Polygon2d& second, double eps)
{
    MultiPolygon2d result = BooleanCompose(first, second, BooleanOp::Difference, eps);
    if (result.IsEmpty() && BoundsOverlap(first, second, eps))
    {
        MultiPolygon2d fallback = TryAxisAlignedBoxDifferenceFallback(first, second, eps);
        if (!fallback.IsEmpty())
        {
            result = std::move(fallback);
        }
    }
    return result;
}
} // namespace geometry::sdk



