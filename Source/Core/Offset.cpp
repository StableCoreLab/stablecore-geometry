#include "Core/Offset.h"

#include <algorithm>
#include <numeric>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "../Detail/RingIntegral2d.h"
#include "../Detail/TolerancePolicy2d.h"
#include "Core/PolygonRelation2d.h"
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

        [[nodiscard]] SCPolyline2d OffsetSegmentPolyline(const SCPolyline2d& polyline,
                                                         double distance,
                                                         double eps)
        {
            if (!polyline.IsValid(eps) || polyline.SegmentCount() == 0)
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
                        const SCLineSegment2d segment = Offset(static_cast<const SCLineSegment2d&>(*source), distance, eps);
                        if (!segment.IsValid(eps))
                        {
                            return SCPolyline2d(polyline.IsClosed() ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
                        }
                        offsetSegment = std::make_shared<SCLineSegment2d>(segment);
                        break;
                    }
                    case SCSegmentKind2::Arc: {
                        const auto& sourceArc = static_cast<const SCArcSegment2d&>(*source);
                        const SCArcSegment2d segment = Offset(sourceArc, distance, eps);
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

                if (!segments.empty() && !segments.back()->EndPoint().AlmostEquals(offsetSegment->StartPoint(), eps))
                {
                    segments.push_back(
                        std::make_shared<SCLineSegment2d>(segments.back()->EndPoint(), offsetSegment->StartPoint()));
                }

                segments.push_back(std::move(offsetSegment));
            }

            if (polyline.IsClosed() && !segments.empty() &&
                !segments.back()->EndPoint().AlmostEquals(segments.front()->StartPoint(), eps))
            {
                segments.push_back(
                    std::make_shared<SCLineSegment2d>(segments.back()->EndPoint(), segments.front()->StartPoint()));
            }

            return SCPolyline2d(std::move(segments),
                              polyline.IsClosed() ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
        }

        [[nodiscard]] double SignedRingArea(const SCPolyline2d& ring, double eps)
        {
            return Detail::ComputeSignedArea(ring, eps);
        }

        [[nodiscard]] SCVector2d LeftNormal(const SCVector2d& direction, double eps = Geometry::kDefaultEpsilon)
        {
            const double length = direction.Length();
            if (length <= eps)
            {
                return SCVector2d{};
            }

            return SCVector2d{-direction.y / length, direction.x / length};
        }

        [[nodiscard]] bool IsZeroVector(const SCVector2d& vector, double eps)
        {
            return vector.LengthSquared() <= eps * eps;
        }

        [[nodiscard]] SCPoint2d IntersectLines(const SCPoint2d& firstPoint,
                                             const SCVector2d& firstDirection,
                                             const SCPoint2d& secondPoint,
                                             const SCVector2d& secondDirection,
                                             bool& success,
                                             double eps)
        {
            const double firstLength = firstDirection.Length();
            const double secondLength = secondDirection.Length();
            if (!(firstLength > eps) || !(secondLength > eps))
            {
                success = false;
                return SCPoint2d{};
            }
            const SCVector2d firstUnit = firstDirection / firstLength;
            const SCVector2d secondUnit = secondDirection / secondLength;
            const double denom = firstUnit.x * secondUnit.y - firstUnit.y * secondUnit.x;
            if (std::abs(denom) <= Geometry::Detail::kParallelDirectionSineEpsilon2d)
            {
                success = false;
                return SCPoint2d{};
            }

            const SCVector2d delta = secondPoint - firstPoint;
            const double t = (delta.x * secondUnit.y - delta.y * secondUnit.x) / denom;
            success = std::isfinite(t);
            return SCPoint2d{firstPoint.x + firstUnit.x * t, firstPoint.y + firstUnit.y * t};
        }

        [[nodiscard]] std::vector<SCPoint2d> BuildOffsetVertices(const std::vector<SCPoint2d>& vertices,
                                                               bool closed,
                                                               double distance,
                                                               const SCOffsetOptions2d& options,
                                                               double epsilon)
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
                normals.push_back(LeftNormal(direction, epsilon));
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
                    SCPoint2d joined = IntersectLines(prevPoint, directions[prev], nextPoint, directions[next], success,
                                                    epsilon);
                    const double maxJoinDistance = std::abs(distance) * std::max(1.0, options.miterLimit);
                    if (!success || Distance(joined, vertices[i]) > maxJoinDistance)
                    {
                        const SCVector2d blended = normals[prev] + normals[next];
                        const SCVector2d direction = IsZeroVector(blended, epsilon) ? normals[next] : blended / blended.Length();
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
                SCPoint2d joined = IntersectLines(prevPoint, directions[prev], nextPoint, directions[next], success,
                                                epsilon);
                const double maxJoinDistance = std::abs(distance) * std::max(1.0, options.miterLimit);
                if (!success || Distance(joined, vertices[i]) > maxJoinDistance)
                {
                    const SCVector2d blended = normals[prev] + normals[next];
                    const SCVector2d direction = IsZeroVector(blended, epsilon) ? normals[next] : blended / blended.Length();
                    joined = vertices[i] + direction * std::min(std::abs(distance), maxJoinDistance);
                }
                result.push_back(joined);
            }

            return result;
        }

        [[nodiscard]] SCPolyline2d BuildPolylineFromVertices(std::vector<SCPoint2d> vertices, bool closed, double eps)
        {
            if (vertices.size() < 2)
            {
                return SCPolyline2d(closed ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
            }

            return Normalize(SCPolyline2d(std::move(vertices), closed ? SCPolylineClosure::Closed : SCPolylineClosure::Open), eps);
        }

        [[nodiscard]] double RingDistance(const SCPolyline2d& ring, double distance, bool isHole, double eps)
        {
            const bool ccw = SignedRingArea(ring, eps) > 0.0;
            const double outward = ccw ? -distance : distance;
            return isHole ? -outward : outward;
        }

        void AppendOffsetRing(const SCPolyline2d& ring, SCMultiPolyline2d& output, double eps)
        {
            const SCPolyline2d normalized = Normalize(ring, eps);
            if (!normalized.IsValid(eps))
            {
                return;
            }
            if (normalized.IsClosed())
            {
                if (normalized.SegmentCount() == 0 ||
                    std::abs(SignedRingArea(normalized, eps)) <=
                        256.0 * eps * eps)
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
                                       double epsilon,
                                       SCMultiPolyline2d& output)
        {
            const std::size_t before = output.Size();
            const SCPolyline2d primary = Offset(ring, signedDistance, options, epsilon);
            AppendOffsetRing(primary, output, epsilon);
            if (output.Size() > before)
            {
                return;
            }

            const SCPolyline2d reversed = Reverse(ring);
            const SCPolyline2d reverseFallback = Offset(reversed, -signedDistance, options, epsilon);
            AppendOffsetRing(reverseFallback, output, epsilon);
            if (output.Size() > before)
            {
                return;
            }

            SCOffsetOptions2d conservativeOptions = options;
            conservativeOptions.miterLimit = std::max(1.0, std::min(options.miterLimit, 2.0));
            const SCPolyline2d conservative = Offset(ring, signedDistance, conservativeOptions, epsilon);
            AppendOffsetRing(conservative, output, epsilon);
        }

        [[nodiscard]] SCPolyline2d NormalizeOuterRing(SCPolyline2d ring, double eps)
        {
            if (SignedRingArea(ring, eps) < 0.0)
            {
                ring = Reverse(ring);
            }
            return ring;
        }

        [[nodiscard]] SCPolyline2d NormalizeHoleRing(SCPolyline2d ring, double eps)
        {
            if (SignedRingArea(ring, eps) > 0.0)
            {
                ring = Reverse(ring);
            }
            return ring;
        }

        [[nodiscard]] SCPolygon2d NormalizePolygonOrientationForOffset(const SCPolygon2d& polygon)
        {
            const SCPolyline2d outer = NormalizeOuterRing(polygon.OuterRing(), Geometry::kDefaultEpsilon);
            std::vector<SCPolyline2d> holes;
            holes.reserve(polygon.HoleCount());
            for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
            {
                holes.push_back(NormalizeHoleRing(polygon.HoleAt(i), Geometry::kDefaultEpsilon));
            }
            return SCPolygon2d(outer, std::move(holes));
        }

        [[nodiscard]] SCMultiPolygon2d BuildOffsetPolygons(const SCMultiPolyline2d& rings,
                                                            double eps,
                                                            bool allowLegacyRetry)
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
            for (std::size_t i = 0; i < rings.Size(); ++i)
            {
                reversedRings.Add(Reverse(rings[i]));
            }
            rebuilt = BuildMultiPolygonByLines(reversedRings, eps);
            if (!rebuilt.IsEmpty())
            {
                return rebuilt;
            }

            if (allowLegacyRetry)
            {
                return BuildMultiPolygonByLines(rings, std::max(eps, Geometry::Detail::kRebuildFallbackEpsilon2d));
            }
            return {};
        }

        [[nodiscard]] SCPoint2d PrimaryReferencePoint(const SCPolygon2d& polygon, double eps)
        {
            const SCPoint2d centroid = polygon.Centroid(eps);
            if (LocatePoint(centroid, polygon, eps) != SCPointContainment2d::Outside)
            {
                return centroid;
            }

            const SCPolyline2d& outer = polygon.OuterRing();
            for (std::size_t i = 0; i < outer.PointCount(); ++i)
            {
                const SCPoint2d start = outer.VertexAt(i);
                const SCPoint2d end = outer.VertexAt((i + 1) % outer.PointCount());
                const SCPoint2d midpoint{std::midpoint(start.x, end.x), std::midpoint(start.y, end.y)};
                if (LocatePoint(midpoint, polygon, eps) != SCPointContainment2d::Outside)
                {
                    return midpoint;
                }
            }

            return outer.VertexAt(0);
        }

        [[nodiscard]] double OffsetAreaTolerance(const SCPolygon2d& source,
                                                   const SCPolygon2d& candidate,
                                                   double eps)
        {
            SCBox2d bounds = source.Bounds(eps);
            bounds.ExpandToInclude(candidate.Bounds(eps));
            if (!bounds.IsValid())
            {
                return eps * eps;
            }
            return std::max(eps * eps, eps * std::hypot(bounds.Width(), bounds.Height()));
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
            double bestScore = -std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < polygons.Size(); ++i)
            {
                const SCPolygon2d& candidate = polygons[i];
                const double areaEpsilon = OffsetAreaTolerance(source, candidate, eps);
                double score = candidate.Area(eps);
                const SCPointContainment2d referenceContainment = LocatePoint(reference, candidate, eps);
                const SCPointContainment2d candidateContainment = LocatePoint(candidate.Centroid(eps), source, eps);
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
                    if (candidate.Area(eps) + areaEpsilon < source.Area(eps))
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
                    if (candidate.Area(eps) > source.Area(eps) + areaEpsilon)
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

        [[nodiscard]] SCPolygon2d SelectLargestPolygon(const SCMultiPolygon2d& polygons, double eps)
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

        [[nodiscard]] SCPolygon2d RecoverOffsetSemanticFlip(const SCPolygon2d& source,
                                                          const SCPolygon2d& candidate,
                                                          double distance,
                                                          double eps)
        {
            if (!candidate.IsValid(eps))
            {
                return candidate;
            }

            const bool outward = distance >= 0.0;
            const double areaEpsilon = OffsetAreaTolerance(source, candidate, eps);
            const double sourceArea = std::abs(source.Area(eps));
            const double candidateArea = std::abs(candidate.Area(eps));
            const PolygonContainment2d relation = Relate(candidate, source, eps);

            if (outward)
            {
                const bool needsRepair =
                    relation == PolygonContainment2d::SecondContainsFirst ||
                    candidateArea + areaEpsilon < sourceArea;
                if (!needsRepair)
                {
                    return candidate;
                }

                const SCMultiPolygon2d repairedCandidates = Union(candidate, source, eps);
                const SCPolygon2d repaired = SelectLargestPolygon(repairedCandidates, eps);
                if (repaired.IsValid(eps) && std::abs(repaired.Area(eps)) + areaEpsilon >= sourceArea)
                {
                    return repaired;
                }

                return candidate;
            }

            const bool needsRepair = relation == PolygonContainment2d::FirstContainsSecond ||
                                     relation == PolygonContainment2d::Disjoint ||
                                     candidateArea > sourceArea + areaEpsilon;
            if (!needsRepair)
            {
                return candidate;
            }

            const SCMultiPolygon2d repairedCandidates = Intersect(candidate, source, eps);
            const SCPolygon2d repaired = SelectLargestPolygon(repairedCandidates, eps);
            if (repaired.IsValid(eps) && std::abs(repaired.Area(eps)) <= sourceArea + areaEpsilon)
            {
                return repaired;
            }

            return candidate;
        }

        [[nodiscard]] bool IsPointInsideAnyPolygon(const SCPoint2d& point, const SCMultiPolygon2d& polygons, double eps)
        {
            for (std::size_t i = 0; i < polygons.Size(); ++i)
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
            if (!candidate.IsValid(eps) || sources.IsEmpty())
            {
                return {};
            }

            const SCPoint2d candidateReference = PrimaryReferencePoint(candidate, eps);
            std::size_t bestIndex = static_cast<std::size_t>(-1);
            double bestScore = -std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < sources.Size(); ++i)
            {
                const SCPolygon2d& source = sources[i];
                if (!source.IsValid(eps))
                {
                    continue;
                }

                double score = -std::abs(std::abs(candidate.Area(eps)) - std::abs(source.Area(eps)));
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
            for (std::size_t i = 0; i < candidates.Size(); ++i)
            {
                const SCPolygon2d& candidate = candidates[i];
                if (!candidate.IsValid(eps))
                {
                    continue;
                }

                const SCPolygon2d bestSource = PickMostRelatedSourcePolygon(candidate, sources, eps);
                SCPolygon2d recovered = candidate;
                if (bestSource.IsValid(eps))
                {
                    recovered = RecoverOffsetSemanticFlip(bestSource, candidate, distance, eps);
                }

                if (!recovered.IsValid(eps))
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
                    for (std::size_t sourceIndex = 0; sourceIndex < sources.Size(); ++sourceIndex)
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
            for (std::size_t sourceIndex = 0; sourceIndex < sources.Size(); ++sourceIndex)
            {
                const SCPolygon2d& source = sources[sourceIndex];
                bool represented = false;
                for (std::size_t candidateIndex = 0; candidateIndex < repaired.Size(); ++candidateIndex)
                {
                    if (Relate(repaired[candidateIndex], source, eps) != PolygonContainment2d::Disjoint)
                    {
                        represented = true;
                        break;
                    }
                }

                if (!represented && source.IsValid(eps))
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
        if (IsZeroVector(normal, Geometry::kDefaultEpsilon))
        {
            return segment;
        }

        return SCLineSegment2d(segment.startPoint + normal * distance, segment.endPoint + normal * distance);
    }

    SCLineSegment2d Offset(const SCLineSegment2d& segment, double distance, double epsilon)
    {
        if (!(epsilon > 0.0) || !std::isfinite(epsilon) || !std::isfinite(distance) ||
            !segment.IsValid(epsilon))
        {
            return {};
        }

        const SCVector2d direction = segment.endPoint - segment.startPoint;
        const double length = direction.Length();
        if (!(length > epsilon) || !std::isfinite(length))
        {
            return {};
        }

        const SCVector2d normal{-direction.y / length, direction.x / length};
        const SCLineSegment2d shifted(segment.startPoint + normal * distance,
                                      segment.endPoint + normal * distance);
        return shifted.IsValid(epsilon) ? shifted : SCLineSegment2d{};
    }

    SCArcSegment2d Offset(const SCArcSegment2d& segment, double distance)
    {
        if (!segment.IsValid())
        {
            return segment;
        }

        const double adjustedRadius = segment.Direction() == SCArcDirection::CounterClockwise ? segment.radius - distance
                                                                                            : segment.radius + distance;
        if (!(adjustedRadius > Geometry::kDefaultEpsilon))
        {
            return SCArcSegment2d{};
        }
        return SCArcSegment2d(segment.center, adjustedRadius, segment.startAngle, segment.sweepAngle);
    }

    SCArcSegment2d Offset(const SCArcSegment2d& segment, double distance, double epsilon)
    {
        if (!(epsilon > 0.0) || !std::isfinite(epsilon) || !std::isfinite(distance) || !segment.IsValid() ||
            !segment.StartPoint().IsValid() || !segment.EndPoint().IsValid())
        {
            return {};
        }

        const double adjustedRadius = segment.Direction() == SCArcDirection::CounterClockwise
                                          ? segment.radius - distance
                                          : segment.radius + distance;
        if (!(adjustedRadius > epsilon) || !std::isfinite(adjustedRadius))
        {
            return {};
        }

        const SCArcSegment2d shifted(segment.center, adjustedRadius, segment.startAngle, segment.sweepAngle);
        return shifted.IsValid() && shifted.StartPoint().IsValid() && shifted.EndPoint().IsValid()
                   ? shifted
                   : SCArcSegment2d{};
    }

    SCPolyline2d Offset(const SCPolyline2d& polyline, double distance, SCOffsetOptions2d options)
    {
        return Offset(polyline, distance, options, Geometry::kDefaultEpsilon);
    }

    SCPolyline2d Offset(const SCPolyline2d& polyline, double distance, SCOffsetOptions2d options, double epsilon)
    {
        if (!(epsilon > 0.0) || !std::isfinite(epsilon))
        {
            return SCPolyline2d(polyline.IsClosed() ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
        }
        if (!polyline.IsValid(epsilon) || polyline.PointCount() < 2)
        {
            return SCPolyline2d(polyline.IsClosed() ? SCPolylineClosure::Closed : SCPolylineClosure::Open);
        }

        if (ContainsArcSegment(polyline))
        {
            return OffsetSegmentPolyline(polyline, distance, epsilon);
        }

        std::vector<SCPoint2d> vertices;
        vertices.reserve(polyline.PointCount());
        for (std::size_t i = 0; i < polyline.PointCount(); ++i)
        {
            vertices.push_back(polyline.VertexAt(i));
        }

        return BuildPolylineFromVertices(BuildOffsetVertices(vertices, polyline.IsClosed(), distance, options, epsilon),
                                         polyline.IsClosed(), epsilon);
    }

    static SCPolygon2d OffsetPolygonImpl(const SCPolygon2d& polygon,
                                         double distance,
                                         SCOffsetOptions2d options,
                                         double epsilon,
                                         bool allowLegacyRetry);

    SCPolygon2d Offset(const SCPolygon2d& polygon, double distance, SCOffsetOptions2d options)
    {
        return OffsetPolygonImpl(polygon, distance, options, Geometry::kDefaultEpsilon, true);
    }

    SCPolygon2d Offset(const SCPolygon2d& polygon, double distance, SCOffsetOptions2d options, double epsilon)
    {
        return OffsetPolygonImpl(polygon, distance, options, epsilon, false);
    }

    static SCPolygon2d OffsetPolygonImpl(const SCPolygon2d& polygon,
                                         double distance,
                                         SCOffsetOptions2d options,
                                         double epsilon,
                                         bool allowLegacyRetry)
    {
        if (!(epsilon > 0.0) || !std::isfinite(epsilon))
        {
            return {};
        }
        SCPolygon2d source = polygon;
        if (allowLegacyRetry)
        {
            source = NormalizePolygonByLines(polygon, epsilon);
            if (!source.IsValid(epsilon))
            {
                source = polygon;
            }
            if (!source.IsValid(epsilon))
            {
                source = NormalizePolygonOrientationForOffset(polygon);
            }
        }

        if (!source.IsValid(epsilon))
        {
            return SCPolygon2d();
        }

        SCMultiPolyline2d offsetRings;
        const SCPolyline2d outerRing = source.OuterRing();
        AppendRecoveredOffsetRing(outerRing, RingDistance(outerRing, distance, false, epsilon), options, epsilon, offsetRings);
        for (std::size_t i = 0; i < source.HoleCount(); ++i)
        {
            const SCPolyline2d hole = source.HoleAt(i);
            AppendRecoveredOffsetRing(hole, RingDistance(hole, distance, true, epsilon), options, epsilon, offsetRings);
        }

        const SCMultiPolygon2d rebuilt = BuildOffsetPolygons(offsetRings, epsilon, allowLegacyRetry);
        if (rebuilt.IsEmpty())
        {
            const SCPolyline2d offsetOuter = Offset(outerRing, RingDistance(outerRing, distance, false, epsilon), options, epsilon);
            if (!offsetOuter.IsValid(epsilon) || !offsetOuter.IsClosed())
            {
                return {};
            }

            std::vector<SCPolyline2d> holes;
            holes.reserve(source.HoleCount());
            for (std::size_t i = 0; i < source.HoleCount(); ++i)
            {
                const SCPolyline2d hole = source.HoleAt(i);
                const SCPolyline2d offsetHole = Offset(hole, RingDistance(hole, distance, true, epsilon), options, epsilon);
                if (!offsetHole.IsValid(epsilon) || !offsetHole.IsClosed())
                {
                    continue;
                }
                holes.push_back(NormalizeHoleRing(offsetHole, epsilon));
            }

            SCPolygon2d fallback(NormalizeOuterRing(offsetOuter, epsilon), std::move(holes));
            if (fallback.IsValid(epsilon))
            {
                return fallback;
            }
            return {};
        }

        const SCPolygon2d selected = SelectBestOffsetPolygon(source, rebuilt, distance, epsilon);
        return RecoverOffsetSemanticFlip(source, selected, distance, epsilon);
    }

    SCMultiPolygon2d OffsetToMultiPolygon(const SCPolygon2d& polygon, double distance, SCOffsetOptions2d options)
    {
        if (!polygon.IsValid())
        {
            return {};
        }
        return Offset(SCMultiPolygon2d{polygon}, distance, options);
    }

    SCMultiPolygon2d OffsetToMultiPolygon(const SCPolygon2d& polygon,
                                         double distance,
                                         SCOffsetOptions2d options,
                                         double epsilon)
    {
        if (!(epsilon > 0.0) || !std::isfinite(epsilon) || !polygon.IsValid(epsilon))
        {
            return {};
        }

        return Offset(SCMultiPolygon2d{polygon}, distance, options, epsilon);
    }

    SCMultiPolyline2d Offset(const SCMultiPolyline2d& polylines, double distance, SCOffsetOptions2d options)
    {
        return Offset(polylines, distance, options, Geometry::kDefaultEpsilon);
    }

    SCMultiPolyline2d Offset(const SCMultiPolyline2d& polylines,
                             double distance,
                             SCOffsetOptions2d options,
                             double epsilon)
    {
        SCMultiPolyline2d result;
        for (std::size_t i = 0; i < polylines.Size(); ++i)
        {
            SCPolyline2d offset = Offset(polylines[i], distance, options, epsilon);
            if (offset.PointCount() >= 2)
            {
                result.Add(std::move(offset));
            }
        }
        return result;
    }

    static SCMultiPolygon2d OffsetMultiPolygonImpl(const SCMultiPolygon2d& polygons,
                                                   double distance,
                                                   SCOffsetOptions2d options,
                                                   double epsilon,
                                                   bool allowLegacyRetry);

    SCMultiPolygon2d Offset(const SCMultiPolygon2d& polygons, double distance, SCOffsetOptions2d options)
    {
        return OffsetMultiPolygonImpl(polygons, distance, options, Geometry::kDefaultEpsilon, true);
    }

    SCMultiPolygon2d Offset(const SCMultiPolygon2d& polygons,
                            double distance,
                            SCOffsetOptions2d options,
                            double epsilon)
    {
        return OffsetMultiPolygonImpl(polygons, distance, options, epsilon, false);
    }

    static SCMultiPolygon2d OffsetMultiPolygonImpl(const SCMultiPolygon2d& polygons,
                                                   double distance,
                                                   SCOffsetOptions2d options,
                                                   double epsilon,
                                                   bool allowLegacyRetry)
    {
        if (!(epsilon > 0.0) || !std::isfinite(epsilon))
        {
            return {};
        }
        SCMultiPolyline2d offsetRings;
        SCMultiPolygon2d normalizedSources;
        for (std::size_t i = 0; i < polygons.Size(); ++i)
        {
            SCPolygon2d source = polygons[i];
            if (allowLegacyRetry)
            {
                source = NormalizePolygonByLines(polygons[i], epsilon);
                if (!source.IsValid(epsilon))
                {
                    source = polygons[i];
                }
                if (!source.IsValid(epsilon))
                {
                    source = NormalizePolygonOrientationForOffset(polygons[i]);
                }
            }
            if (!source.IsValid(epsilon))
            {
                continue;
            }

            normalizedSources.Add(SCPolygon2d(source));

            const SCPolyline2d outerRing = source.OuterRing();
            AppendRecoveredOffsetRing(outerRing, RingDistance(outerRing, distance, false, epsilon), options, epsilon, offsetRings);
            for (std::size_t holeIndex = 0; holeIndex < source.HoleCount(); ++holeIndex)
            {
                const SCPolyline2d hole = source.HoleAt(holeIndex);
                AppendRecoveredOffsetRing(hole, RingDistance(hole, distance, true, epsilon), options, epsilon, offsetRings);
            }
        }

        const SCMultiPolygon2d rebuilt = BuildOffsetPolygons(offsetRings, epsilon, allowLegacyRetry);
        return RecoverMultiPolygonSemanticFlip(normalizedSources, rebuilt, distance, epsilon);
    }
}  // namespace Geometry

