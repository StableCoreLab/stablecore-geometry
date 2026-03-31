#include "sdk/GeometryOffset.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "common/Epsilon.h"
#include "sdk/GeometryEditing.h"
#include "sdk/GeometryMetrics.h"
#include "sdk/GeometryPathOps.h"
#include "sdk/GeometryRelation.h"
#include "sdk/GeometryShapeOps.h"
#include "sdk/GeometryTopology.h"
#include "sdk/GeometryValidation.h"

namespace geometry::sdk
{
namespace
{
[[nodiscard]] Vector2d LeftNormal(const Vector2d& direction)
{
    const double length = direction.Length();
    if (length <= geometry::kDefaultEpsilon)
    {
        return Vector2d{};
    }

    return Vector2d{-direction.y / length, direction.x / length};
}

[[nodiscard]] bool IsZeroVector(const Vector2d& vector)
{
    return vector.LengthSquared() <= geometry::kDefaultEpsilon * geometry::kDefaultEpsilon;
}

[[nodiscard]] Point2d IntersectLines(
    const Point2d& firstPoint,
    const Vector2d& firstDirection,
    const Point2d& secondPoint,
    const Vector2d& secondDirection,
    bool& success)
{
    const double denom = firstDirection.x * secondDirection.y - firstDirection.y * secondDirection.x;
    if (std::abs(denom) <= geometry::kDefaultEpsilon)
    {
        success = false;
        return Point2d{};
    }

    const Vector2d delta = secondPoint - firstPoint;
    const double t = (delta.x * secondDirection.y - delta.y * secondDirection.x) / denom;
    success = std::isfinite(t);
    return Point2d{
        firstPoint.x + firstDirection.x * t,
        firstPoint.y + firstDirection.y * t};
}

[[nodiscard]] std::vector<Point2d> BuildOffsetVertices(
    const std::vector<Point2d>& vertices,
    bool closed,
    double distance,
    const OffsetOptions2d& options)
{
    std::vector<Point2d> result;
    if (vertices.size() < 2)
    {
        return result;
    }

    const std::size_t count = vertices.size();
    const std::size_t segmentCount = closed ? count : count - 1;

    std::vector<Vector2d> directions;
    directions.reserve(segmentCount);
    std::vector<Vector2d> normals;
    normals.reserve(segmentCount);
    for (std::size_t i = 0; i < segmentCount; ++i)
    {
        const Point2d& start = vertices[i];
        const Point2d& end = vertices[(i + 1) % count];
        const Vector2d direction = end - start;
        directions.push_back(direction);
        normals.push_back(LeftNormal(direction));
    }

    result.reserve(count);
    if (!closed)
    {
        result.push_back(vertices.front() + normals.front() * distance);
        for (std::size_t i = 1; i + 1 < count; ++i)
        {
            const std::size_t prev = i - 1;
            const std::size_t next = i;
            const Point2d prevPoint = vertices[i] + normals[prev] * distance;
            const Point2d nextPoint = vertices[i] + normals[next] * distance;
            bool success = false;
            Point2d joined = IntersectLines(
                prevPoint,
                directions[prev],
                nextPoint,
                directions[next],
                success);
            const double maxJoinDistance = std::abs(distance) * std::max(1.0, options.miterLimit);
            if (!success || Distance(joined, vertices[i]) > maxJoinDistance)
            {
                const Vector2d blended = normals[prev] + normals[next];
                const Vector2d direction =
                    IsZeroVector(blended) ? normals[next] : blended / blended.Length();
                joined = vertices[i] + direction * std::min(std::abs(distance), maxJoinDistance);
            }
            result.push_back(joined);
        }
        result.push_back(vertices.back() + normals.back() * distance);
        return result;
    }

    for (std::size_t i = 0; i < count; ++i)
    {
        const std::size_t prev = (i + count - 1) % count;
        const std::size_t next = i % segmentCount;
        const Point2d prevPoint = vertices[i] + normals[prev] * distance;
        const Point2d nextPoint = vertices[i] + normals[next] * distance;
        bool success = false;
        Point2d joined = IntersectLines(
            prevPoint,
            directions[prev],
            nextPoint,
            directions[next],
            success);
        const double maxJoinDistance = std::abs(distance) * std::max(1.0, options.miterLimit);
        if (!success || Distance(joined, vertices[i]) > maxJoinDistance)
        {
            const Vector2d blended = normals[prev] + normals[next];
            const Vector2d direction =
                IsZeroVector(blended) ? normals[next] : blended / blended.Length();
            joined = vertices[i] + direction * std::min(std::abs(distance), maxJoinDistance);
        }
        result.push_back(joined);
    }

    return result;
}

[[nodiscard]] Polyline2d BuildPolylineFromVertices(std::vector<Point2d> vertices, bool closed)
{
    if (vertices.size() < 2)
    {
        return Polyline2d(closed ? PolylineClosure::Closed : PolylineClosure::Open);
    }

    return Normalize(
        Polyline2d(std::move(vertices), closed ? PolylineClosure::Closed : PolylineClosure::Open));
}

[[nodiscard]] double RingDistance(const Polyline2d& ring, double distance, bool isHole)
{
    const bool ccw = Orientation(ring) == RingOrientation2d::CounterClockwise;
    const double outward = ccw ? -distance : distance;
    return isHole ? -outward : outward;
}

void AppendOffsetRing(const Polyline2d& ring, MultiPolyline2d& output)
{
    const Polyline2d normalized = Normalize(ring, geometry::kDefaultEpsilon);
    if (!normalized.IsValid())
    {
        return;
    }
    if (normalized.IsClosed())
    {
        if (normalized.PointCount() < 3 ||
            std::abs(Area(Polygon2d(normalized))) <= 256.0 * geometry::kDefaultEpsilon * geometry::kDefaultEpsilon)
        {
            return;
        }
    }
    else if (normalized.PointCount() < 2)
    {
        return;
    }

    output.Add(normalized);
}

void AppendRecoveredOffsetRing(
    const Polyline2d& ring,
    double signedDistance,
    const OffsetOptions2d& options,
    MultiPolyline2d& output)
{
    const std::size_t before = output.Count();
    const Polyline2d primary = Offset(ring, signedDistance, options);
    AppendOffsetRing(primary, output);
    if (output.Count() > before)
    {
        return;
    }

    const Polyline2d reversed = Reverse(ring);
    const Polyline2d reverseFallback = Offset(reversed, -signedDistance, options);
    AppendOffsetRing(reverseFallback, output);
    if (output.Count() > before)
    {
        return;
    }

    OffsetOptions2d conservativeOptions = options;
    conservativeOptions.miterLimit = std::max(1.0, std::min(options.miterLimit, 2.0));
    const Polyline2d conservative = Offset(ring, signedDistance, conservativeOptions);
    AppendOffsetRing(conservative, output);
}

[[nodiscard]] Polyline2d NormalizeOuterRing(Polyline2d ring)
{
    if (Orientation(ring) != RingOrientation2d::CounterClockwise)
    {
        ring = Reverse(ring);
    }
    return ring;
}

[[nodiscard]] Polyline2d NormalizeHoleRing(Polyline2d ring)
{
    if (Orientation(ring) != RingOrientation2d::Clockwise)
    {
        ring = Reverse(ring);
    }
    return ring;
}

[[nodiscard]] Polygon2d NormalizePolygonOrientationForOffset(const Polygon2d& polygon)
{
    const Polyline2d outer = NormalizeOuterRing(polygon.OuterRing());
    std::vector<Polyline2d> holes;
    holes.reserve(polygon.HoleCount());
    for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
    {
        holes.push_back(NormalizeHoleRing(polygon.HoleAt(i)));
    }
    return Polygon2d(outer, std::move(holes));
}

[[nodiscard]] MultiPolygon2d BuildOffsetPolygons(const MultiPolyline2d& rings, double eps)
{
    if (rings.IsEmpty())
    {
        return {};
    }

    MultiPolygon2d rebuilt = BuildMultiPolygonByLines(rings, eps);
    if (!rebuilt.IsEmpty())
    {
        return rebuilt;
    }

    MultiPolyline2d reversedRings;
    for (std::size_t i = 0; i < rings.Count(); ++i)
    {
        reversedRings.Add(Reverse(rings[i]));
    }
    rebuilt = BuildMultiPolygonByLines(reversedRings, eps);
    if (!rebuilt.IsEmpty())
    {
        return rebuilt;
    }

    return BuildMultiPolygonByLines(rings, std::max(eps, 1e-8));
}

[[nodiscard]] Point2d PrimaryReferencePoint(const Polygon2d& polygon, double eps)
{
    const Point2d centroid = Centroid(polygon);
    if (LocatePoint(centroid, polygon, eps) != PointContainment2d::Outside)
    {
        return centroid;
    }

    const Polyline2d& outer = polygon.OuterRing();
    for (std::size_t i = 0; i < outer.PointCount(); ++i)
    {
        const Point2d start = outer.PointAt(i);
        const Point2d end = outer.PointAt((i + 1) % outer.PointCount());
        const Point2d midpoint{0.5 * (start.x + end.x), 0.5 * (start.y + end.y)};
        if (LocatePoint(midpoint, polygon, eps) != PointContainment2d::Outside)
        {
            return midpoint;
        }
    }

    return outer.PointAt(0);
}

[[nodiscard]] Polygon2d SelectBestOffsetPolygon(const Polygon2d& source, const MultiPolygon2d& polygons, double distance, double eps)
{
    if (polygons.IsEmpty())
    {
        return {};
    }

    const Point2d reference = PrimaryReferencePoint(source, eps);
    const bool outward = distance >= 0.0;
    std::size_t bestIndex = 0;
    double bestScore = -1.0;
    for (std::size_t i = 0; i < polygons.Count(); ++i)
    {
        const Polygon2d& candidate = polygons[i];
        double score = Area(candidate);
        const PointContainment2d referenceContainment = LocatePoint(reference, candidate, eps);
        const PointContainment2d candidateContainment = LocatePoint(Centroid(candidate), source, eps);
        const PolygonContainment2d relation = Relate(candidate, source, eps);
        if (outward)
        {
            if (referenceContainment != PointContainment2d::Outside)
            {
                score += 1e9;
            }
            if (relation == PolygonContainment2d::FirstContainsSecond)
            {
                score += 1e9;
            }
            else if (relation == PolygonContainment2d::Touching || relation == PolygonContainment2d::Intersecting)
            {
                score += 1e7;
            }
            else if (relation == PolygonContainment2d::Disjoint)
            {
                score -= 1e7;
            }
            if (Area(candidate) + eps < Area(source))
            {
                score -= 1e8;
            }
        }
        else
        {
            if (candidateContainment != PointContainment2d::Outside)
            {
                score += 1e9;
            }
            if (referenceContainment != PointContainment2d::Outside)
            {
                score += 1e6;
            }
            if (relation == PolygonContainment2d::SecondContainsFirst)
            {
                score += 1e9;
            }
            else if (relation == PolygonContainment2d::Touching || relation == PolygonContainment2d::Intersecting)
            {
                score += 1e7;
            }
            if (Area(candidate) > Area(source) + eps)
            {
                score -= 1e7;
            }
        }

        score -= static_cast<double>(std::abs(static_cast<int>(candidate.HoleCount()) - static_cast<int>(source.HoleCount()))) *
                 1e4;

        if (score > bestScore)
        {
            bestScore = score;
            bestIndex = i;
        }
    }

    return polygons[bestIndex];
}
} // namespace

LineSegment2d Offset(const LineSegment2d& segment, double distance)
{
    if (!segment.IsValid())
    {
        return segment;
    }

    const Vector2d direction = segment.endPoint - segment.startPoint;
    const Vector2d normal = LeftNormal(direction);
    if (IsZeroVector(normal))
    {
        return segment;
    }

    return LineSegment2d(segment.startPoint + normal * distance, segment.endPoint + normal * distance);
}

ArcSegment2d Offset(const ArcSegment2d& segment, double distance)
{
    if (!segment.IsValid())
    {
        return segment;
    }

    const double adjustedRadius =
        segment.Direction() == ArcDirection::CounterClockwise ? segment.radius - distance
                                                              : segment.radius + distance;
    if (!(adjustedRadius > geometry::kDefaultEpsilon))
    {
        return ArcSegment2d{};
    }
    return ArcSegment2d(segment.center, adjustedRadius, segment.startAngle, segment.sweepAngle);
}

Polyline2d Offset(const Polyline2d& polyline, double distance, OffsetOptions2d options)
{
    if (!polyline.IsValid() || polyline.PointCount() < 2)
    {
        return Polyline2d(polyline.IsClosed() ? PolylineClosure::Closed : PolylineClosure::Open);
    }

    std::vector<Point2d> vertices;
    vertices.reserve(polyline.PointCount());
    for (std::size_t i = 0; i < polyline.PointCount(); ++i)
    {
        vertices.push_back(polyline.PointAt(i));
    }

    return BuildPolylineFromVertices(
        BuildOffsetVertices(vertices, polyline.IsClosed(), distance, options),
        polyline.IsClosed());
}

Polygon2d Offset(const Polygon2d& polygon, double distance, OffsetOptions2d options)
{
    Polygon2d source = polygon;
    if (!source.IsValid())
    {
        source = NormalizePolygonOrientationForOffset(polygon);
    }

    if (!source.IsValid())
    {
        return Polygon2d();
    }

    MultiPolyline2d offsetRings;
    const Polyline2d outerRing = source.OuterRing();
    AppendRecoveredOffsetRing(outerRing, RingDistance(outerRing, distance, false), options, offsetRings);
    for (std::size_t i = 0; i < source.HoleCount(); ++i)
    {
        const Polyline2d hole = source.HoleAt(i);
        AppendRecoveredOffsetRing(hole, RingDistance(hole, distance, true), options, offsetRings);
    }

    const MultiPolygon2d rebuilt = BuildOffsetPolygons(offsetRings, geometry::kDefaultEpsilon);
    if (rebuilt.IsEmpty())
    {
        const Polyline2d offsetOuter = Offset(outerRing, RingDistance(outerRing, distance, false), options);
        if (!offsetOuter.IsValid() || !offsetOuter.IsClosed())
        {
            return {};
        }

        std::vector<Polyline2d> holes;
        holes.reserve(source.HoleCount());
        for (std::size_t i = 0; i < source.HoleCount(); ++i)
        {
            const Polyline2d hole = source.HoleAt(i);
            const Polyline2d offsetHole = Offset(hole, RingDistance(hole, distance, true), options);
            if (!offsetHole.IsValid() || !offsetHole.IsClosed())
            {
                continue;
            }
            holes.push_back(NormalizeHoleRing(offsetHole));
        }

        Polygon2d fallback(NormalizeOuterRing(offsetOuter), std::move(holes));
        if (fallback.IsValid())
        {
            return fallback;
        }
        return {};
    }

    return SelectBestOffsetPolygon(source, rebuilt, distance, geometry::kDefaultEpsilon);
}

MultiPolyline2d Offset(const MultiPolyline2d& polylines, double distance, OffsetOptions2d options)
{
    MultiPolyline2d result;
    for (std::size_t i = 0; i < polylines.Count(); ++i)
    {
        Polyline2d offset = Offset(polylines[i], distance, options);
        if (offset.PointCount() >= 2)
        {
            result.Add(std::move(offset));
        }
    }
    return result;
}

MultiPolygon2d Offset(const MultiPolygon2d& polygons, double distance, OffsetOptions2d options)
{
    MultiPolyline2d offsetRings;
    for (std::size_t i = 0; i < polygons.Count(); ++i)
    {
        const Polygon2d& polygon = polygons[i];
        if (!polygon.IsValid())
        {
            continue;
        }

        const Polyline2d outerRing = polygon.OuterRing();
        AppendRecoveredOffsetRing(outerRing, RingDistance(outerRing, distance, false), options, offsetRings);
        for (std::size_t holeIndex = 0; holeIndex < polygon.HoleCount(); ++holeIndex)
        {
            const Polyline2d hole = polygon.HoleAt(holeIndex);
            AppendRecoveredOffsetRing(hole, RingDistance(hole, distance, true), options, offsetRings);
        }
    }

    return BuildOffsetPolygons(offsetRings, geometry::kDefaultEpsilon);
}
} // namespace geometry::sdk
