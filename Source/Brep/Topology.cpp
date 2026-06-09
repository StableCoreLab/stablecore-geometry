#include "Brep/Topology.h"

#include <sstream>
#include <vector>

#include "Core/Intersection.h"
#include "Core/Relation.h"
#include "Geometry2d/SCPathOps.h"

namespace Geometry
{
    namespace
    {
        enum class BoundaryContact2d
        {
            None,
            Touching,
            Overlapping,
            Crossing
        };

        void CollectRingSegments(const SCPolyline2d& ring, std::vector<SCLineSegment2d>& segments)
        {
            if (!ring.IsClosed() || ring.PointCount() < 2)
            {
                return;
            }

            for (std::size_t i = 0; i < ring.PointCount(); ++i)
            {
                segments.emplace_back(ring.PointAt(i), ring.PointAt((i + 1) % ring.PointCount()));
            }
        }

        [[nodiscard]] std::vector<SCLineSegment2d> CollectPolygonSegments(const SCPolygon2d& polygon)
        {
            std::vector<SCLineSegment2d> segments;
            CollectRingSegments(polygon.OuterRing(), segments);
            for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
            {
                CollectRingSegments(polygon.HoleAt(i), segments);
            }
            return segments;
        }

        [[nodiscard]] bool IsEndpointParameter(double parameter, double eps)
        {
            return std::abs(parameter) <= eps || std::abs(parameter - 1.0) <= eps;
        }

        [[nodiscard]] BoundaryContact2d ClassifyBoundaryContact(const SCPolygon2d& first,
                                                                const SCPolygon2d& second,
                                                                double eps)
        {
            const std::vector<SCLineSegment2d> firstSegments = CollectPolygonSegments(first);
            const std::vector<SCLineSegment2d> secondSegments = CollectPolygonSegments(second);

            BoundaryContact2d contact = BoundaryContact2d::None;
            for (const SCLineSegment2d& lhs : firstSegments)
            {
                for (const SCLineSegment2d& rhs : secondSegments)
                {
                    const SCSegmentIntersection2d intersection = Intersect(lhs, rhs, eps);
                    if (!intersection.HasIntersection())
                    {
                        continue;
                    }
                    if (intersection.kind == SCIntersectionKind2d::Overlap)
                    {
                        contact = BoundaryContact2d::Overlapping;
                        continue;
                    }

                    for (std::size_t i = 0; i < intersection.pointCount; ++i)
                    {
                        const bool endpointOnFirst = IsEndpointParameter(intersection.points[i].parameterOnFirst, eps);
                        const bool endpointOnSecond =
                            IsEndpointParameter(intersection.points[i].parameterOnSecond, eps);
                        if (!endpointOnFirst || !endpointOnSecond)
                        {
                            return BoundaryContact2d::Crossing;
                        }
                        contact = BoundaryContact2d::Touching;
                    }
                }
            }

            return contact;
        }

        [[nodiscard]] bool TryFindStrictInteriorSample(const SCPolygon2d& polygon, double eps, SCPoint2d& sample)
        {
            const SCPoint2d centroid = Centroid(polygon);
            if (LocatePoint(centroid, polygon, eps) == SCPointContainment2d::Inside)
            {
                sample = centroid;
                return true;
            }

            const SCPolyline2d ring = polygon.OuterRing();
            if (!ring.IsValid() || ring.PointCount() < 3)
            {
                return false;
            }

            const SCRingOrientation2d orientation = Orientation(ring);
            for (std::size_t i = 0; i < ring.PointCount(); ++i)
            {
                const SCPoint2d start = ring.PointAt(i);
                const SCPoint2d end = ring.PointAt((i + 1) % ring.PointCount());
                const SCVector2d edge = end - start;
                const double length = edge.Length();
                if (length <= eps)
                {
                    continue;
                }

                SCVector2d inward{-edge.y / length, edge.x / length};
                if (orientation == SCRingOrientation2d::Clockwise)
                {
                    inward = inward * -1.0;
                }

                const SCPoint2d midpoint{0.5 * (start.x + end.x), 0.5 * (start.y + end.y)};
                const double step = std::max(16.0 * eps, std::min(0.2 * length, 1e-3));
                const SCPoint2d candidatePoint = midpoint + inward * step;
                if (LocatePoint(candidatePoint, polygon, eps) == SCPointContainment2d::Inside)
                {
                    sample = candidatePoint;
                    return true;
                }
            }

            return false;
        }

        [[nodiscard]] bool HasStrictInteriorPoint(const SCPolygon2d& container, const SCPolygon2d& candidate, double eps)
        {
            SCPoint2d strictInterior{};
            if (TryFindStrictInteriorSample(candidate, eps, strictInterior) &&
                LocatePoint(strictInterior, container, eps) == SCPointContainment2d::Inside)
            {
                return true;
            }

            const SCPolyline2d& ring = candidate.OuterRing();
            for (std::size_t i = 0; i < ring.PointCount(); ++i)
            {
                const SCPoint2d point = ring.PointAt(i);
                if (LocatePoint(point, container, eps) == SCPointContainment2d::Inside)
                {
                    return true;
                }

                const SCPoint2d next = ring.PointAt((i + 1) % ring.PointCount());
                const SCPoint2d midpoint{0.5 * (point.x + next.x), 0.5 * (point.y + next.y)};
                if (LocatePoint(midpoint, container, eps) == SCPointContainment2d::Inside)
                {
                    return true;
                }
            }
            return false;
        }
    }  // namespace

    PolygonTopology2d::PolygonTopology2d(const SCMultiPolygon2d& polygons, double eps)
    {
        Build(polygons, eps);
    }

    bool PolygonTopology2d::Build(const SCMultiPolygon2d& polygons, double eps)
    {
        polygons_.Clear();
        for (std::size_t i = 0; i < polygons.Count(); ++i)
        {
            SCPolygon2d normalized = NormalizePolygonByLines(polygons[i], eps);
            if (!normalized.IsValid())
            {
                normalized = polygons[i];
            }
            polygons_.Add(std::move(normalized));
        }
        nodes_.clear();
        roots_.clear();
        nodes_.resize(polygons_.Count());
        for (std::size_t i = 0; i < polygons_.Count(); ++i)
        {
            nodes_[i].polygonIndex = i;
            std::size_t bestParent = static_cast<std::size_t>(-1);
            double bestArea = 0.0;
            for (std::size_t j = 0; j < polygons_.Count(); ++j)
            {
                if (i == j)
                {
                    continue;
                }

                const PolygonContainment2d relation = Relate(polygons_[j], polygons_[i], eps);
                if (relation != PolygonContainment2d::FirstContainsSecond &&
                    !(relation == PolygonContainment2d::Equal && j < i))
                {
                    continue;
                }

                const double area = polygons_[j].Area();
                if (bestParent == static_cast<std::size_t>(-1) || area < bestArea)
                {
                    bestParent = j;
                    bestArea = area;
                }
            }
            nodes_[i].parentIndex = bestParent;
        }

        for (std::size_t i = 0; i < nodes_.size(); ++i)
        {
            if (nodes_[i].parentIndex == static_cast<std::size_t>(-1))
            {
                roots_.push_back(i);
            } else
            {
                nodes_[nodes_[i].parentIndex].children.push_back(i);
            }
        }
        return true;
    }

    std::size_t PolygonTopology2d::Count() const
    {
        return polygons_.Count();
    }
    bool PolygonTopology2d::IsEmpty() const
    {
        return polygons_.IsEmpty();
    }
    bool PolygonTopology2d::IsValid() const
    {
        return nodes_.size() == polygons_.Count();
    }
    const SCMultiPolygon2d& PolygonTopology2d::Polygons() const
    {
        return polygons_;
    }
    const PolygonTopologyNode2d& PolygonTopology2d::Node(std::size_t index) const
    {
        return nodes_.at(index);
    }
    const std::vector<std::size_t>& PolygonTopology2d::Roots() const
    {
        return roots_;
    }
    const std::vector<std::size_t>& PolygonTopology2d::ChildrenOf(std::size_t index) const
    {
        return nodes_.at(index).children;
    }
    std::size_t PolygonTopology2d::ParentOf(std::size_t index) const
    {
        return nodes_.at(index).parentIndex;
    }

    std::string PolygonTopology2d::DebugString() const
    {
        std::ostringstream stream;
        stream << "PolygonTopology2d{polygonCount=" << Count() << ", rootCount=" << roots_.size() << "}";
        return stream.str();
    }

    bool ContainsPoint(const SCPolyline2d& ring, const SCPoint2d& point, double eps)
    {
        const SCPointContainment2d containment = LocatePoint(point, ring, eps);
        return containment == SCPointContainment2d::Inside || containment == SCPointContainment2d::OnBoundary;
    }

    bool ContainsPoint(const SCPolygon2d& polygon, const SCPoint2d& point, double eps)
    {
        const SCPointContainment2d containment = LocatePoint(point, polygon, eps);
        return containment == SCPointContainment2d::Inside || containment == SCPointContainment2d::OnBoundary;
    }

    bool Contains(const SCPolygon2d& outer, const SCPolygon2d& inner, double eps)
    {
        SCPolygon2d normalizedOuter = NormalizePolygonByLines(outer, eps);
        if (!normalizedOuter.IsValid())
        {
            normalizedOuter = outer;
        }

        SCPolygon2d normalizedInner = NormalizePolygonByLines(inner, eps);
        if (!normalizedInner.IsValid())
        {
            normalizedInner = inner;
        }

        if (!normalizedOuter.IsValid() || !normalizedInner.IsValid())
        {
            return false;
        }

        for (std::size_t i = 0; i < normalizedInner.OuterRing().PointCount(); ++i)
        {
            if (LocatePoint(normalizedInner.OuterRing().PointAt(i), normalizedOuter, eps) ==
                SCPointContainment2d::Outside)
            {
                return false;
            }
        }

        SCPoint2d strictInnerSample{};
        if (TryFindStrictInteriorSample(normalizedInner, eps, strictInnerSample) &&
            LocatePoint(strictInnerSample, normalizedOuter, eps) == SCPointContainment2d::Outside)
        {
            return false;
        }

        // If an outer-hole interior point lies inside the candidate,
        // containment cannot hold even when the two outer boundaries coincide.
        for (std::size_t holeIndex = 0; holeIndex < normalizedOuter.HoleCount(); ++holeIndex)
        {
            SCPolyline2d holeRing = normalizedOuter.HoleAt(holeIndex);
            SCPolygon2d holeRegion(holeRing);
            if (!holeRegion.IsValid())
            {
                holeRegion = SCPolygon2d(Reverse(holeRing));
            }
            if (!holeRegion.IsValid())
            {
                continue;
            }

            SCPoint2d holeSample{};
            if (TryFindStrictInteriorSample(holeRegion, eps, holeSample) &&
                LocatePoint(holeSample, normalizedInner, eps) != SCPointContainment2d::Outside)
            {
                return false;
            }
        }

        return true;
    }

    PolygonContainment2d Relate(const SCPolygon2d& first, const SCPolygon2d& second, double eps)
    {
        SCPolygon2d normalizedFirst = NormalizePolygonByLines(first, eps);
        if (!normalizedFirst.IsValid())
        {
            normalizedFirst = first;
        }

        SCPolygon2d normalizedSecond = NormalizePolygonByLines(second, eps);
        if (!normalizedSecond.IsValid())
        {
            normalizedSecond = second;
        }

        if (!normalizedFirst.IsValid() || !normalizedSecond.IsValid())
        {
            return PolygonContainment2d::Disjoint;
        }

        const bool firstContainsSecond = Contains(normalizedFirst, normalizedSecond, eps);
        const bool secondContainsFirst = Contains(normalizedSecond, normalizedFirst, eps);
        const bool secondHasStrictInteriorInFirst = HasStrictInteriorPoint(normalizedFirst, normalizedSecond, eps);
        const bool firstHasStrictInteriorInSecond = HasStrictInteriorPoint(normalizedSecond, normalizedFirst, eps);
        if (firstContainsSecond && secondContainsFirst)
        {
            return PolygonContainment2d::Equal;
        }
        if (firstContainsSecond)
        {
            return secondHasStrictInteriorInFirst ? PolygonContainment2d::FirstContainsSecond
                                                  : PolygonContainment2d::Touching;
        }
        if (secondContainsFirst)
        {
            return firstHasStrictInteriorInSecond ? PolygonContainment2d::SecondContainsFirst
                                                  : PolygonContainment2d::Touching;
        }

        const BoundaryContact2d contact = ClassifyBoundaryContact(normalizedFirst, normalizedSecond, eps);
        if (contact == BoundaryContact2d::Crossing)
        {
            return PolygonContainment2d::Intersecting;
        }
        if (contact == BoundaryContact2d::Overlapping)
        {
            return (secondHasStrictInteriorInFirst || firstHasStrictInteriorInSecond)
                       ? PolygonContainment2d::Intersecting
                       : PolygonContainment2d::Touching;
        }
        if (contact == BoundaryContact2d::Touching)
        {
            return (secondHasStrictInteriorInFirst || firstHasStrictInteriorInSecond)
                       ? PolygonContainment2d::Intersecting
                       : PolygonContainment2d::Touching;
        }

        if (secondHasStrictInteriorInFirst || firstHasStrictInteriorInSecond)
        {
            return PolygonContainment2d::Intersecting;
        }

        return PolygonContainment2d::Disjoint;
    }

    PolygonTopology2d BuildPolygonTopology(const SCMultiPolygon2d& polygons, double eps)
    {
        return PolygonTopology2d(polygons, eps);
    }
}  // namespace Geometry
