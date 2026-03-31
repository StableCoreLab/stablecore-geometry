#include "sdk/GeometryTopology.h"

#include <sstream>
#include <vector>

#include "sdk/GeometryIntersection.h"
#include "sdk/GeometryRelation.h"

namespace geometry::sdk
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

void CollectRingSegments(const Polyline2d& ring, std::vector<LineSegment2d>& segments)
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

[[nodiscard]] std::vector<LineSegment2d> CollectPolygonSegments(const Polygon2d& polygon)
{
    std::vector<LineSegment2d> segments;
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

[[nodiscard]] BoundaryContact2d ClassifyBoundaryContact(
    const Polygon2d& first,
    const Polygon2d& second,
    double eps)
{
    const std::vector<LineSegment2d> firstSegments = CollectPolygonSegments(first);
    const std::vector<LineSegment2d> secondSegments = CollectPolygonSegments(second);

    BoundaryContact2d contact = BoundaryContact2d::None;
    for (const LineSegment2d& lhs : firstSegments)
    {
        for (const LineSegment2d& rhs : secondSegments)
        {
            const SegmentIntersection2d intersection = Intersect(lhs, rhs, eps);
            if (!intersection.HasIntersection())
            {
                continue;
            }
            if (intersection.kind == IntersectionKind2d::Overlap)
            {
                contact = BoundaryContact2d::Overlapping;
                continue;
            }

            for (std::size_t i = 0; i < intersection.pointCount; ++i)
            {
                const bool endpointOnFirst = IsEndpointParameter(intersection.points[i].parameterOnFirst, eps);
                const bool endpointOnSecond = IsEndpointParameter(intersection.points[i].parameterOnSecond, eps);
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

[[nodiscard]] bool HasStrictInteriorPoint(const Polygon2d& container, const Polygon2d& candidate, double eps)
{
    const Point2d candidateCentroid = Centroid(candidate);
    if (LocatePoint(candidateCentroid, container, eps) == PointContainment2d::Inside)
    {
        return true;
    }

    const Polyline2d& ring = candidate.OuterRing();
    for (std::size_t i = 0; i < ring.PointCount(); ++i)
    {
        const Point2d point = ring.PointAt(i);
        if (LocatePoint(point, container, eps) == PointContainment2d::Inside)
        {
            return true;
        }

        const Point2d next = ring.PointAt((i + 1) % ring.PointCount());
        const Point2d midpoint{0.5 * (point.x + next.x), 0.5 * (point.y + next.y)};
        if (LocatePoint(midpoint, container, eps) == PointContainment2d::Inside)
        {
            return true;
        }
    }
    return false;
}
} // namespace

PolygonTopology2d::PolygonTopology2d(const MultiPolygon2d& polygons, double eps)
{
    Build(polygons, eps);
}

bool PolygonTopology2d::Build(const MultiPolygon2d& polygons, double eps)
{
    polygons_ = polygons;
    nodes_.clear();
    roots_.clear();
    nodes_.resize(polygons.Count());
    for (std::size_t i = 0; i < polygons.Count(); ++i)
    {
        nodes_[i].polygonIndex = i;
        std::size_t bestParent = static_cast<std::size_t>(-1);
        double bestArea = 0.0;
        for (std::size_t j = 0; j < polygons.Count(); ++j)
        {
            if (i == j)
            {
                continue;
            }

            const PolygonContainment2d relation = Relate(polygons[j], polygons[i], eps);
            if (relation != PolygonContainment2d::FirstContainsSecond && !(relation == PolygonContainment2d::Equal && j < i))
            {
                continue;
            }

            const double area = Area(polygons[j]);
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
        }
        else
        {
            nodes_[nodes_[i].parentIndex].children.push_back(i);
        }
    }
    return true;
}

std::size_t PolygonTopology2d::Count() const { return polygons_.Count(); }
bool PolygonTopology2d::IsEmpty() const { return polygons_.IsEmpty(); }
bool PolygonTopology2d::IsValid() const { return nodes_.size() == polygons_.Count(); }
const MultiPolygon2d& PolygonTopology2d::Polygons() const { return polygons_; }
const PolygonTopologyNode2d& PolygonTopology2d::Node(std::size_t index) const { return nodes_.at(index); }
const std::vector<std::size_t>& PolygonTopology2d::Roots() const { return roots_; }
const std::vector<std::size_t>& PolygonTopology2d::ChildrenOf(std::size_t index) const { return nodes_.at(index).children; }
std::size_t PolygonTopology2d::ParentOf(std::size_t index) const { return nodes_.at(index).parentIndex; }

std::string PolygonTopology2d::DebugString() const
{
    std::ostringstream stream;
    stream << "PolygonTopology2d{polygonCount=" << Count() << ", rootCount=" << roots_.size() << "}";
    return stream.str();
}

bool ContainsPoint(const Polyline2d& ring, const Point2d& point, double eps)
{
    const PointContainment2d containment = LocatePoint(point, ring, eps);
    return containment == PointContainment2d::Inside || containment == PointContainment2d::OnBoundary;
}

bool ContainsPoint(const Polygon2d& polygon, const Point2d& point, double eps)
{
    const PointContainment2d containment = LocatePoint(point, polygon, eps);
    return containment == PointContainment2d::Inside || containment == PointContainment2d::OnBoundary;
}

bool Contains(const Polygon2d& outer, const Polygon2d& inner, double eps)
{
    if (!outer.IsValid() || !inner.IsValid())
    {
        return false;
    }

    if (ClassifyBoundaryContact(outer, inner, eps) == BoundaryContact2d::Crossing)
    {
        return false;
    }

    for (std::size_t i = 0; i < inner.OuterRing().PointCount(); ++i)
    {
        if (LocatePoint(inner.OuterRing().PointAt(i), outer, eps) == PointContainment2d::Outside)
        {
            return false;
        }
    }

    return true;
}

PolygonContainment2d Relate(const Polygon2d& first, const Polygon2d& second, double eps)
{
    if (!first.IsValid() || !second.IsValid())
    {
        return PolygonContainment2d::Disjoint;
    }

    const bool firstContainsSecond = Contains(first, second, eps);
    const bool secondContainsFirst = Contains(second, first, eps);
    const bool secondHasStrictInteriorInFirst = HasStrictInteriorPoint(first, second, eps);
    const bool firstHasStrictInteriorInSecond = HasStrictInteriorPoint(second, first, eps);
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

    const BoundaryContact2d contact = ClassifyBoundaryContact(first, second, eps);
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

PolygonTopology2d BuildPolygonTopology(const MultiPolygon2d& polygons, double eps)
{
    return PolygonTopology2d(polygons, eps);
}
} // namespace geometry::sdk

