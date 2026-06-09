#include "Core/Offset.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

#include "RingIntegral2d.h"
#include "Brep/Topology.h"
#include "Core/Boolean.h"
#include "Core/Editing.h"
#include "Core/Metrics.h"
#include "Core/Relation.h"
#include "Core/ShapeOps.h"
#include "Core/Validation.h"
#include "Geometry2d/SCPathOps.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    namespace
    {
        [[nodiscard]] bool ContainsArcSegment(const SCPolyline2d& polyline)
        {
            for (std::size_t i = 0; i < polyline.SegmentCount(); ++i)
            {
                std::unique_ptr<ISCSegment2d> segment = polyline.SegmentAt(i);
                if (segment != nullptr && segment->Kind() == SCSegmentKind2::Arc)
                {
                    return true;
                }
            }

            return false;
        }

        [[nodiscard]] SCPolyline2d OffsetSegmentPolyline(const SCPolyline2d& polyline, double distance)
        {
            if (!polyline.IsValid() || polyline.SegmentCount() == 0)
            {
                return SCPolyline2d(polyline.IsClosed() ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
            }

            std::vector<std::shared_ptr<ISCSegment2d>> segments;
            segments.reserve(polyline.SegmentCount() * 2 + (polyline.IsClosed() ? 1 : 0));
            for (std::size_t i = 0; i < polyline.SegmentCount(); ++i)
            {
                std::unique_ptr<ISCSegment2d> source = polyline.SegmentAt(i);
                if (source == nullptr)
                {
                    return SCPolyline2d(polyline.IsClosed() ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
                }

                std::shared_ptr<ISCSegment2d> offsetSegment;
                switch (source->Kind())
                {
                    case SCSegmentKind2::Line: {
                        const SCLineSegment2d segment = Offset(static_cast<const SCLineSegment2d&>(*source), distance);
                        if (!segment.IsValid())
                        {
                            return SCPolyline2d(polyline.IsClosed() ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
                        }
                        offsetSegment = std::make_shared<SCLineSegment2d>(segment);
                        break;
                    }
                    case SCSegmentKind2::Arc: {
                        const SCArcSegment2d segment = Offset(static_cast<const SCArcSegment2d&>(*source), distance);
                        if (!segment.IsValid())
                        {
                            return SCPolyline2d(polyline.IsClosed() ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
                        }
                        offsetSegment = std::make_shared<SCArcSegment2d>(segment);
                        break;
                    }
                    default:
                        return SCPolyline2d(polyline.IsClosed() ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
                }

                if (!segments.empty() && !segments.back()->EndPoint().AlmostEquals(offsetSegment->StartPoint(),
                                                                                   Geometry::kOffsetDefaultEpsilon))
                {
                    segments.push_back(
                        std::make_shared<SCLineSegment2d>(segments.back()->EndPoint(), offsetSegment->StartPoint()));
                }

                segments.push_back(std::move(offsetSegment));
            }

            if (polyline.IsClosed() && !segments.empty() &&
                !segments.back()->EndPoint().AlmostEquals(segments.front()->StartPoint(),
                                                          Geometry::kOffsetDefaultEpsilon))
            {
                segments.push_back(
                    std::make_shared<SCLineSegment2d>(segments.back()->EndPoint(), segments.front()->StartPoint()));
            }

            return SCPolyline2d(std::move(segments),
                              polyline.IsClosed() ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
        }

        [[nodiscard]] double SignedRingArea(const SCPolyline2d& ring)
        {
            return Detail::ComputeSignedArea(ring);
        }

        [[nodiscard]] SCVector2d LeftNormal(const SCVector2d& direction)
        {
            const double length = direction.Length();
            if (length <= Geometry::kOffsetDefaultEpsilon)
            {
                return SCVector2d{};
            }

            return SCVector2d{-direction.y / length, direction.x / length};
        }

        [[nodiscard]] bool IsZeroVector(const SCVector2d& vector)
        {
            return vector.LengthSquared() <= Geometry::kOffsetDefaultEpsilon * Geometry::kOffsetDefaultEpsilon;
        }

        [[nodiscard]] SCPoint2d IntersectLines(const SCPoint2d& firstPoint,
                                             const SCVector2d& firstDirection,
                                             const SCPoint2d& secondPoint,
                                             const SCVector2d& secondDirection,
                                             bool& success)
        {
            const double denom = firstDirection.x * secondDirection.y - firstDirection.y * secondDirection.x;
            if (std::abs(denom) <= Geometry::kOffsetDefaultEpsilon)
            {
                success = false;
                return SCPoint2d{};
            }

            const SCVector2d delta = secondPoint - firstPoint;
            const double t = (delta.x * secondDirection.y - delta.y * secondDirection.x) / denom;
            success = std::isfinite(t);
            return SCPoint2d{firstPoint.x + firstDirection.x * t, firstPoint.y + firstDirection.y * t};
        }

        [[nodiscard]] std::vector<SCPoint2d> BuildOffsetVertices(const std::vector<SCPoint2d>& vertices,
                                                               bool closed,
                                                               double distance,
                                                               const SCOffsetOptions2d& options)
        {
            std::vector<SCPoint2d> result;
            if (vertices.size() < 2)
            {
                return result;
            }

            const std::size_t count = vertices.size();
            const std::size_t segmentCount = closed ? count : count - 1;

            std::vector<SCVector2d> directions;
            directions.reserve(segmentCount);
            std::vector<SCVector2d> normals;
            normals.reserve(segmentCount);
            for (std::size_t i = 0; i < segmentCount; ++i)
            {
                const SCPoint2d& start = vertices[i];
                const SCPoint2d& end = vertices[(i + 1) % count];
                const SCVector2d direction = end - start;
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
                    const SCPoint2d prevPoint = vertices[i] + normals[prev] * distance;
                    const SCPoint2d nextPoint = vertices[i] + normals[next] * distance;
                    bool success = false;
                    SCPoint2d joined = IntersectLines(prevPoint, directions[prev], nextPoint, directions[next], success);
                    const double maxJoinDistance = std::abs(distance) * std::max(1.0, options.miterLimit);
                    if (!success || Distance(joined, vertices[i]) > maxJoinDistance)
                    {
                        const SCVector2d blended = normals[prev] + normals[next];
                        const SCVector2d direction = IsZeroVector(blended) ? normals[next] : blended / blended.Length();
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
                const SCPoint2d prevPoint = vertices[i] + normals[prev] * distance;
                const SCPoint2d nextPoint = vertices[i] + normals[next] * distance;
                bool success = false;
                SCPoint2d joined = IntersectLines(prevPoint, directions[prev], nextPoint, directions[next], success);
                const double maxJoinDistance = std::abs(distance) * std::max(1.0, options.miterLimit);
                if (!success || Distance(joined, vertices[i]) > maxJoinDistance)
                {
                    const SCVector2d blended = normals[prev] + normals[next];
                    const SCVector2d direction = IsZeroVector(blended) ? normals[next] : blended / blended.Length();
                    joined = vertices[i] + direction * std::min(std::abs(distance), maxJoinDistance);
                }
                result.push_back(joined);
            }

            return result;
        }

        [[nodiscard]] SCPolyline2d BuildPolylineFromVertices(std::vector<SCPoint2d> vertices, bool closed)
        {
            if (vertices.size() < 2)
            {
                return SCPolyline2d(closed ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
            }

            return Normalize(SCPolyline2d(std::move(vertices), closed ? SCPolylineClosure::Closed : SCPolylineClosure::Open));
        }

        [[nodiscard]] double RingDistance(const SCPolyline2d& ring, double distance, bool isHole)
        {
            const bool ccw = Orientation(ring) == SCRingOrientation2d::CounterClockwise;
            const double outward = ccw ? -distance : distance;
            return isHole ? -outward : outward;
        }

        void AppendOffsetRing(const SCPolyline2d& ring, SCMultiPolyline2d& output)
        {
            const SCPolyline2d normalized = Normalize(ring, Geometry::kOffsetDefaultEpsilon);
            if (!normalized.IsValid())
            {
                return;
            }
            if (normalized.IsClosed())
            {
                if (normalized.SegmentCount() == 0 ||
                    std::abs(SignedRingArea(normalized)) <=
                        256.0 * Geometry::kOffsetDefaultEpsilon * Geometry::kOffsetDefaultEpsilon)
                {
                    return;
                }
            } else if (normalized.PointCount() < 2)
            {
                return;
            }

            output.Add(normalized);
        }

        void AppendRecoveredOffsetRing(const SCPolyline2d& ring,
                                       double signedDistance,
                                       const SCOffsetOptions2d& options,
                                       SCMultiPolyline2d& output)
        {
            const std::size_t before = output.Count();
            const SCPolyline2d primary = Offset(ring, signedDistance, options);
            AppendOffsetRing(primary, output);
            if (output.Count() > before)
            {
                return;
            }

            const SCPolyline2d reversed = Reverse(ring);
            const SCPolyline2d reverseFallback = Offset(reversed, -signedDistance, options);
            AppendOffsetRing(reverseFallback, output);
            if (output.Count() > before)
            {
                return;
            }

            SCOffsetOptions2d conservativeOptions = options;
            conservativeOptions.miterLimit = std::max(1.0, std::min(options.miterLimit, 2.0));
            const SCPolyline2d conservative = Offset(ring, signedDistance, conservativeOptions);
            AppendOffsetRing(conservative, output);
        }

        [[nodiscard]] SCPolyline2d NormalizeOuterRing(SCPolyline2d ring)
        {
            if (Orientation(ring) != SCRingOrientation2d::CounterClockwise)
            {
                ring = Reverse(ring);
            }
            return ring;
        }

        [[nodiscard]] SCPolyline2d NormalizeHoleRing(SCPolyline2d ring)
        {
            if (Orientation(ring) != SCRingOrientation2d::Clockwise)
            {
                ring = Reverse(ring);
            }
            return ring;
        }

        [[nodiscard]] SCPolygon2d NormalizePolygonOrientationForOffset(const SCPolygon2d& polygon)
        {
            const SCPolyline2d outer = NormalizeOuterRing(polygon.OuterRing());
            std::vector<SCPolyline2d> holes;
            holes.reserve(polygon.HoleCount());
            for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
            {
                holes.push_back(NormalizeHoleRing(polygon.HoleAt(i)));
            }
            return SCPolygon2d(outer, std::move(holes));
        }

        [[nodiscard]] SCMultiPolygon2d BuildOffsetPolygons(const SCMultiPolyline2d& rings, double eps)
        {
            if (rings.IsEmpty())
            {
                return {};
            }

            SCMultiPolygon2d rebuilt = BuildMultiPolygonByLines(rings, eps);
            if (!rebuilt.IsEmpty())
            {
                return rebuilt;
            }

            SCMultiPolyline2d reversedRings;
            for (std::size_t i = 0; i < rings.Count(); ++i)
            {
                reversedRings.Add(Reverse(rings[i]));
            }
            rebuilt = BuildMultiPolygonByLines(reversedRings, eps);
            if (!rebuilt.IsEmpty())
            {
                return rebuilt;
            }

            return BuildMultiPolygonByLines(rings, std::max(eps, Geometry::kOffsetRebuildFallbackEpsilon));
        }

        [[nodiscard]] SCPoint2d PrimaryReferencePoint(const SCPolygon2d& polygon, double eps)
        {
            const SCPoint2d centroid = Centroid(polygon);
            if (LocatePoint(centroid, polygon, eps) != SCPointContainment2d::Outside)
            {
                return centroid;
            }

            const SCPolyline2d& outer = polygon.OuterRing();
            for (std::size_t i = 0; i < outer.PointCount(); ++i)
            {
                const SCPoint2d start = outer.PointAt(i);
                const SCPoint2d end = outer.PointAt((i + 1) % outer.PointCount());
                const SCPoint2d midpoint{0.5 * (start.x + end.x), 0.5 * (start.y + end.y)};
                if (LocatePoint(midpoint, polygon, eps) != SCPointContainment2d::Outside)
                {
                    return midpoint;
                }
            }

            return outer.PointAt(0);
        }

        [[nodiscard]] SCPolygon2d SelectBestOffsetPolygon(const SCPolygon2d& source,
                                                        const SCMultiPolygon2d& polygons,
                                                        double distance,
                                                        double eps)
        {
            if (polygons.IsEmpty())
            {
                return {};
            }

            const SCPoint2d reference = PrimaryReferencePoint(source, eps);
            const bool outward = distance >= 0.0;
            std::size_t bestIndex = 0;
            double bestScore = -1.0;
            for (std::size_t i = 0; i < polygons.Count(); ++i)
            {
                const SCPolygon2d& candidate = polygons[i];
                double score = candidate.Area();
                const SCPointContainment2d referenceContainment = LocatePoint(reference, candidate, eps);
                const SCPointContainment2d candidateContainment = LocatePoint(Centroid(candidate), source, eps);
                const PolygonContainment2d relation = Relate(candidate, source, eps);
                if (outward)
                {
                    if (referenceContainment != SCPointContainment2d::Outside)
                    {
                        score += 1e9;
                    }
                    if (relation == PolygonContainment2d::FirstContainsSecond)
                    {
                        score += 1e9;
                    } else if (relation == PolygonContainment2d::Touching ||
                               relation == PolygonContainment2d::Intersecting)
                    {
                        score += 1e7;
                    } else if (relation == PolygonContainment2d::Disjoint)
                    {
                        score -= 1e7;
                    }
                    if (candidate.Area() + eps < source.Area())
                    {
                        score -= 1e8;
                    }
                } else
                {
                    if (candidateContainment != SCPointContainment2d::Outside)
                    {
                        score += 1e9;
                    }
                    if (referenceContainment != SCPointContainment2d::Outside)
                    {
                        score += 1e6;
                    }
                    if (relation == PolygonContainment2d::SecondContainsFirst)
                    {
                        score += 1e9;
                    } else if (relation == PolygonContainment2d::Touching ||
                               relation == PolygonContainment2d::Intersecting)
                    {
                        score += 1e7;
                    }
                    if (candidate.Area() > source.Area() + eps)
                    {
                        score -= 1e7;
                    }
                }

                score -= static_cast<double>(
                             std::abs(static_cast<int>(candidate.HoleCount()) - static_cast<int>(source.HoleCount()))) *
                         1e4;

                if (score > bestScore)
                {
                    bestScore = score;
                    bestIndex = i;
                }
            }

            return polygons[bestIndex];
        }

        [[nodiscard]] SCPolygon2d SelectLargestPolygon(const SCMultiPolygon2d& polygons)
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

        [[nodiscard]] SCPolygon2d RecoverOffsetSemanticFlip(const SCPolygon2d& source,
                                                          const SCPolygon2d& candidate,
                                                          double distance,
                                                          double eps)
        {
            if (!candidate.IsValid())
            {
                return candidate;
            }

            const bool outward = distance >= 0.0;
            const double sourceArea = std::abs(source.Area());
            const double candidateArea = std::abs(candidate.Area());
            const PolygonContainment2d relation = Relate(candidate, source, eps);

            if (outward)
            {
                const bool needsRepair =
                    relation == PolygonContainment2d::SecondContainsFirst || candidateArea + eps < sourceArea;
                if (!needsRepair)
                {
                    return candidate;
                }

                const SCMultiPolygon2d repairedCandidates = Union(candidate, source, eps);
                const SCPolygon2d repaired = SelectLargestPolygon(repairedCandidates);
                if (repaired.IsValid() && std::abs(repaired.Area()) + eps >= sourceArea)
                {
                    return repaired;
                }

                return candidate;
            }

            const bool needsRepair = relation == PolygonContainment2d::FirstContainsSecond ||
                                     relation == PolygonContainment2d::Disjoint || candidateArea > sourceArea + eps;
            if (!needsRepair)
            {
                return candidate;
            }

            const SCMultiPolygon2d repairedCandidates = Intersect(candidate, source, eps);
            const SCPolygon2d repaired = SelectLargestPolygon(repairedCandidates);
            if (repaired.IsValid() && std::abs(repaired.Area()) <= sourceArea + eps)
            {
                return repaired;
            }

            return candidate;
        }

        [[nodiscard]] bool IsPointInsideAnyPolygon(const SCPoint2d& point, const SCMultiPolygon2d& polygons, double eps)
        {
            for (std::size_t i = 0; i < polygons.Count(); ++i)
            {
                if (LocatePoint(point, polygons[i], eps) != SCPointContainment2d::Outside)
                {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] SCPolygon2d PickMostRelatedSourcePolygon(const SCPolygon2d& candidate,
                                                             const SCMultiPolygon2d& sources,
                                                             double eps)
        {
            if (!candidate.IsValid() || sources.IsEmpty())
            {
                return {};
            }

            const SCPoint2d candidateReference = PrimaryReferencePoint(candidate, eps);
            std::size_t bestIndex = static_cast<std::size_t>(-1);
            double bestScore = -1e100;
            for (std::size_t i = 0; i < sources.Count(); ++i)
            {
                const SCPolygon2d& source = sources[i];
                if (!source.IsValid())
                {
                    continue;
                }

                double score = -std::abs(std::abs(candidate.Area()) - std::abs(source.Area()));
                if (LocatePoint(candidateReference, source, eps) != SCPointContainment2d::Outside)
                {
                    score += 1e9;
                }

                const PolygonContainment2d relation = Relate(candidate, source, eps);
                if (relation == PolygonContainment2d::Equal)
                {
                    score += 1e10;
                } else if (relation == PolygonContainment2d::Touching || relation == PolygonContainment2d::Intersecting)
                {
                    score += 1e7;
                } else if (relation == PolygonContainment2d::FirstContainsSecond ||
                           relation == PolygonContainment2d::SecondContainsFirst)
                {
                    score += 1e6;
                }

                if (score > bestScore)
                {
                    bestScore = score;
                    bestIndex = i;
                }
            }

            if (bestIndex == static_cast<std::size_t>(-1))
            {
                return {};
            }

            return sources[bestIndex];
        }

        [[nodiscard]] SCMultiPolygon2d RecoverMultiPolygonSemanticFlip(const SCMultiPolygon2d& sources,
                                                                     const SCMultiPolygon2d& candidates,
                                                                     double distance,
                                                                     double eps)
        {
            if (candidates.IsEmpty() || sources.IsEmpty())
            {
                return candidates;
            }

            const bool outward = distance >= 0.0;
            SCMultiPolygon2d repaired;
            for (std::size_t i = 0; i < candidates.Count(); ++i)
            {
                const SCPolygon2d& candidate = candidates[i];
                if (!candidate.IsValid())
                {
                    continue;
                }

                const SCPolygon2d bestSource = PickMostRelatedSourcePolygon(candidate, sources, eps);
                SCPolygon2d recovered = candidate;
                if (bestSource.IsValid())
                {
                    recovered = RecoverOffsetSemanticFlip(bestSource, candidate, distance, eps);
                }

                if (!recovered.IsValid())
                {
                    continue;
                }

                const SCPoint2d reference = PrimaryReferencePoint(recovered, eps);
                const bool insideAnySource = IsPointInsideAnyPolygon(reference, sources, eps);
                if (!outward && !insideAnySource)
                {
                    continue;
                }

                if (outward)
                {
                    bool related = false;
                    for (std::size_t sourceIndex = 0; sourceIndex < sources.Count(); ++sourceIndex)
                    {
                        const PolygonContainment2d relation = Relate(recovered, sources[sourceIndex], eps);
                        if (relation != PolygonContainment2d::Disjoint)
                        {
                            related = true;
                            break;
                        }
                    }
                    if (!related)
                    {
                        continue;
                    }
                }

                repaired.Add(std::move(recovered));
            }

            if (!outward)
            {
                return repaired;
            }

            // For outward offsets, keep at least one related result per source
            // polygon to avoid rare semantic-flip shrink where a source
            // component disappears.
            for (std::size_t sourceIndex = 0; sourceIndex < sources.Count(); ++sourceIndex)
            {
                const SCPolygon2d& source = sources[sourceIndex];
                bool represented = false;
                for (std::size_t candidateIndex = 0; candidateIndex < repaired.Count(); ++candidateIndex)
                {
                    if (Relate(repaired[candidateIndex], source, eps) != PolygonContainment2d::Disjoint)
                    {
                        represented = true;
                        break;
                    }
                }

                if (!represented && source.IsValid())
                {
                    repaired.Add(SCPolygon2d(source));
                }
            }

            return repaired;
        }
    }  // namespace

    SCLineSegment2d Offset(const SCLineSegment2d& segment, double distance)
    {
        if (!segment.IsValid())
        {
            return segment;
        }

        const SCVector2d direction = segment.endPoint - segment.startPoint;
        const SCVector2d normal = LeftNormal(direction);
        if (IsZeroVector(normal))
        {
            return segment;
        }

        return SCLineSegment2d(segment.startPoint + normal * distance, segment.endPoint + normal * distance);
    }

    SCArcSegment2d Offset(const SCArcSegment2d& segment, double distance)
    {
        if (!segment.IsValid())
        {
            return segment;
        }

        const double adjustedRadius = segment.Direction() == SCArcDirection::CounterClockwise ? segment.radius - distance
                                                                                            : segment.radius + distance;
        if (!(adjustedRadius > Geometry::kOffsetDefaultEpsilon))
        {
            return SCArcSegment2d{};
        }
        return SCArcSegment2d(segment.center, adjustedRadius, segment.startAngle, segment.sweepAngle);
    }

    SCPolyline2d Offset(const SCPolyline2d& polyline, double distance, SCOffsetOptions2d options)
    {
        if (!polyline.IsValid() || polyline.PointCount() < 2)
        {
            return SCPolyline2d(polyline.IsClosed() ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
        }

        if (ContainsArcSegment(polyline))
        {
            return OffsetSegmentPolyline(polyline, distance);
        }

        std::vector<SCPoint2d> vertices;
        vertices.reserve(polyline.PointCount());
        for (std::size_t i = 0; i < polyline.PointCount(); ++i)
        {
            vertices.push_back(polyline.PointAt(i));
        }

        return BuildPolylineFromVertices(BuildOffsetVertices(vertices, polyline.IsClosed(), distance, options),
                                         polyline.IsClosed());
    }

    SCPolygon2d Offset(const SCPolygon2d& polygon, double distance, SCOffsetOptions2d options)
    {
        SCPolygon2d source = NormalizePolygonByLines(polygon, Geometry::kOffsetDefaultEpsilon);
        if (!source.IsValid())
        {
            source = polygon;
        }
        if (!source.IsValid())
        {
            source = NormalizePolygonOrientationForOffset(polygon);
        }

        if (!source.IsValid())
        {
            return SCPolygon2d();
        }

        SCMultiPolyline2d offsetRings;
        const SCPolyline2d outerRing = source.OuterRing();
        AppendRecoveredOffsetRing(outerRing, RingDistance(outerRing, distance, false), options, offsetRings);
        for (std::size_t i = 0; i < source.HoleCount(); ++i)
        {
            const SCPolyline2d hole = source.HoleAt(i);
            AppendRecoveredOffsetRing(hole, RingDistance(hole, distance, true), options, offsetRings);
        }

        const SCMultiPolygon2d rebuilt = BuildOffsetPolygons(offsetRings, Geometry::kOffsetDefaultEpsilon);
        if (rebuilt.IsEmpty())
        {
            const SCPolyline2d offsetOuter = Offset(outerRing, RingDistance(outerRing, distance, false), options);
            if (!offsetOuter.IsValid() || !offsetOuter.IsClosed())
            {
                return {};
            }

            std::vector<SCPolyline2d> holes;
            holes.reserve(source.HoleCount());
            for (std::size_t i = 0; i < source.HoleCount(); ++i)
            {
                const SCPolyline2d hole = source.HoleAt(i);
                const SCPolyline2d offsetHole = Offset(hole, RingDistance(hole, distance, true), options);
                if (!offsetHole.IsValid() || !offsetHole.IsClosed())
                {
                    continue;
                }
                holes.push_back(NormalizeHoleRing(offsetHole));
            }

            SCPolygon2d fallback(NormalizeOuterRing(offsetOuter), std::move(holes));
            if (fallback.IsValid())
            {
                return fallback;
            }
            return {};
        }

        const SCPolygon2d selected = SelectBestOffsetPolygon(source, rebuilt, distance, Geometry::kOffsetDefaultEpsilon);
        return RecoverOffsetSemanticFlip(source, selected, distance, Geometry::kOffsetDefaultEpsilon);
    }

    SCMultiPolygon2d OffsetToMultiPolygon(const SCPolygon2d& polygon, double distance, SCOffsetOptions2d options)
    {
        if (!polygon.IsValid())
        {
            return {};
        }

        return Offset(SCMultiPolygon2d{polygon}, distance, options);
    }

    SCMultiPolyline2d Offset(const SCMultiPolyline2d& polylines, double distance, SCOffsetOptions2d options)
    {
        SCMultiPolyline2d result;
        for (std::size_t i = 0; i < polylines.Count(); ++i)
        {
            SCPolyline2d offset = Offset(polylines[i], distance, options);
            if (offset.PointCount() >= 2)
            {
                result.Add(std::move(offset));
            }
        }
        return result;
    }

    SCMultiPolygon2d Offset(const SCMultiPolygon2d& polygons, double distance, SCOffsetOptions2d options)
    {
        SCMultiPolyline2d offsetRings;
        SCMultiPolygon2d normalizedSources;
        for (std::size_t i = 0; i < polygons.Count(); ++i)
        {
            SCPolygon2d source = NormalizePolygonByLines(polygons[i], Geometry::kOffsetDefaultEpsilon);
            if (!source.IsValid())
            {
                source = polygons[i];
            }
            if (!source.IsValid())
            {
                source = NormalizePolygonOrientationForOffset(polygons[i]);
            }
            if (!source.IsValid())
            {
                continue;
            }

            normalizedSources.Add(SCPolygon2d(source));

            const SCPolyline2d outerRing = source.OuterRing();
            AppendRecoveredOffsetRing(outerRing, RingDistance(outerRing, distance, false), options, offsetRings);
            for (std::size_t holeIndex = 0; holeIndex < source.HoleCount(); ++holeIndex)
            {
                const SCPolyline2d hole = source.HoleAt(holeIndex);
                AppendRecoveredOffsetRing(hole, RingDistance(hole, distance, true), options, offsetRings);
            }
        }

        const SCMultiPolygon2d rebuilt = BuildOffsetPolygons(offsetRings, Geometry::kOffsetDefaultEpsilon);
        return RecoverMultiPolygonSemanticFlip(normalizedSources, rebuilt, distance, Geometry::kOffsetDefaultEpsilon);
    }
}  // namespace Geometry

