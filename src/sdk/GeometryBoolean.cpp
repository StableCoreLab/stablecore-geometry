#include "sdk/GeometryBoolean.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "algorithm/Predicate2.h"
#include "common/Epsilon.h"
#include "sdk/GeometryIntersection.h"
#include "sdk/GeometryRelation.h"
#include "sdk/GeometryShapeOps.h"
#include "sdk/GeometryTopology.h"

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

struct LinearEdge
{
    Point2d start{};
    Point2d end{};

    [[nodiscard]] bool IsVertical(double eps = geometry::kDefaultEpsilon) const
    {
        return std::abs(end.x - start.x) <= eps;
    }

    [[nodiscard]] bool SpansX(double x, double eps = geometry::kDefaultEpsilon) const
    {
        const double minX = std::min(start.x, end.x);
        const double maxX = std::max(start.x, end.x);
        return x >= minX - eps && x < maxX - eps;
    }

    [[nodiscard]] double YAt(double x) const
    {
        const double dx = end.x - start.x;
        if (std::abs(dx) <= geometry::kDefaultEpsilon)
        {
            return std::min(start.y, end.y);
        }

        const double t = (x - start.x) / dx;
        return start.y + (end.y - start.y) * t;
    }
};

[[nodiscard]] std::vector<LinearEdge> PolygonEdges(const Polygon2d& polygon)
{
    std::vector<LinearEdge> edges;
    auto appendRing = [&edges](const Polyline2d& ring) {
        for (std::size_t i = 0; i < ring.PointCount(); ++i)
        {
            edges.push_back(LinearEdge{
                ring.PointAt(i),
                ring.PointAt((i + 1) % ring.PointCount())});
        }
    };

    appendRing(polygon.OuterRing());
    for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
    {
        appendRing(polygon.HoleAt(i));
    }
    return edges;
}

[[nodiscard]] std::vector<double> CollectEvents(const Polygon2d& first, const Polygon2d& second)
{
    std::vector<double> xs;
    const auto collectVertices = [&xs](const Polygon2d& polygon) {
        auto collectRing = [&xs](const Polyline2d& ring) {
            for (std::size_t i = 0; i < ring.PointCount(); ++i)
            {
                xs.push_back(ring.PointAt(i).x);
            }
        };

        collectRing(polygon.OuterRing());
        for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
        {
            collectRing(polygon.HoleAt(i));
        }
    };

    collectVertices(first);
    collectVertices(second);

    const std::vector<LinearEdge> firstEdges = PolygonEdges(first);
    const std::vector<LinearEdge> secondEdges = PolygonEdges(second);
    for (const auto& lhs : firstEdges)
    {
        const LineSegment2d lhsSegment(lhs.start, lhs.end);
        for (const auto& rhs : secondEdges)
        {
            const LineSegment2d rhsSegment(rhs.start, rhs.end);
            const auto intersection = Intersect(lhsSegment, rhsSegment);
            for (std::size_t i = 0; i < intersection.pointCount; ++i)
            {
                xs.push_back(intersection.points[i].point.x);
            }
        }
    }

    std::sort(xs.begin(), xs.end());
    xs.erase(
        std::unique(xs.begin(), xs.end(), [](double lhs, double rhs) {
            return std::abs(lhs - rhs) <= geometry::kDefaultEpsilon;
        }),
        xs.end());
    return xs;
}

struct ActiveCrossing
{
    LinearEdge edge{};
    double yMid{0.0};
};

[[nodiscard]] std::vector<ActiveCrossing> ActiveCrossings(
    const std::vector<LinearEdge>& edges,
    double xMid)
{
    std::vector<ActiveCrossing> crossings;
    for (const auto& edge : edges)
    {
        if (edge.IsVertical())
        {
            continue;
        }
        if (edge.SpansX(xMid))
        {
            crossings.push_back(ActiveCrossing{edge, edge.YAt(xMid)});
        }
    }

    std::sort(crossings.begin(), crossings.end(), [](const ActiveCrossing& lhs, const ActiveCrossing& rhs) {
        if (std::abs(lhs.yMid - rhs.yMid) <= geometry::kDefaultEpsilon)
        {
            return lhs.edge.start.y < rhs.edge.start.y;
        }
        return lhs.yMid < rhs.yMid;
    });
    return crossings;
}

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

[[nodiscard]] Polygon2d MakeCellPolygon(
    const LinearEdge& lower,
    const LinearEdge& upper,
    double xLeft,
    double xRight)
{
    std::vector<Point2d> points{
        Point2d{xLeft, lower.YAt(xLeft)},
        Point2d{xRight, lower.YAt(xRight)},
        Point2d{xRight, upper.YAt(xRight)},
        Point2d{xLeft, upper.YAt(xLeft)}};

    Polyline2d ring(std::move(points), PolylineClosure::Closed);
    if (Orientation(ring) == RingOrientation2d::Clockwise)
    {
        ring = Reverse(ring);
    }
    return Polygon2d(ring);
}

[[nodiscard]] MultiPolygon2d BooleanCompose(
    const Polygon2d& first,
    const Polygon2d& second,
    BooleanOp op)
{
    MultiPolygon2d result;
    if (!first.IsValid() || !second.IsValid())
    {
        return result;
    }

    const std::vector<double> events = CollectEvents(first, second);
    if (events.size() < 2)
    {
        if (Evaluate(op, true, Contains(second, first)))
        {
            result.Add(Polygon2d(first));
        }
        else if (op == BooleanOp::Union)
        {
            result.Add(Polygon2d(first));
            if (!Contains(first, second))
            {
                result.Add(Polygon2d(second));
            }
        }
        return result;
    }

    const std::vector<LinearEdge> firstEdges = PolygonEdges(first);
    const std::vector<LinearEdge> secondEdges = PolygonEdges(second);

    for (std::size_t i = 0; i + 1 < events.size(); ++i)
    {
        const double xLeft = events[i];
        const double xRight = events[i + 1];
        if (xRight <= xLeft + geometry::kDefaultEpsilon)
        {
            continue;
        }

        const double xMid = 0.5 * (xLeft + xRight);
        std::vector<ActiveCrossing> crossings = ActiveCrossings(firstEdges, xMid);
        {
            std::vector<ActiveCrossing> secondCrossings = ActiveCrossings(secondEdges, xMid);
            crossings.insert(crossings.end(), secondCrossings.begin(), secondCrossings.end());
        }
        std::sort(crossings.begin(), crossings.end(), [](const ActiveCrossing& lhs, const ActiveCrossing& rhs) {
            if (std::abs(lhs.yMid - rhs.yMid) <= geometry::kDefaultEpsilon)
            {
                return lhs.edge.start.y < rhs.edge.start.y;
            }
            return lhs.yMid < rhs.yMid;
        });

        for (std::size_t k = 0; k + 1 < crossings.size(); ++k)
        {
            const double yLower = crossings[k].yMid;
            const double yUpper = crossings[k + 1].yMid;
            if (yUpper <= yLower + geometry::kDefaultEpsilon)
            {
                continue;
            }

            const Point2d sample{xMid, 0.5 * (yLower + yUpper)};
            const bool inFirst = LocatePoint(sample, first) == PointContainment2d::Inside;
            const bool inSecond = LocatePoint(sample, second) == PointContainment2d::Inside;
            if (!Evaluate(op, inFirst, inSecond))
            {
                continue;
            }

            Polygon2d cell = MakeCellPolygon(crossings[k].edge, crossings[k + 1].edge, xLeft, xRight);
            if (cell.IsValid() && Area(cell) > geometry::kDefaultEpsilon)
            {
                result.Add(std::move(cell));
            }
        }
    }

    return result;
}
} // namespace

MultiPolygon2d Intersect(const Polygon2d& first, const Polygon2d& second, double eps)
{
    (void)eps;
    return BooleanCompose(first, second, BooleanOp::Intersection);
}

MultiPolygon2d Union(const Polygon2d& first, const Polygon2d& second, double eps)
{
    (void)eps;
    return BooleanCompose(first, second, BooleanOp::Union);
}

MultiPolygon2d Difference(const Polygon2d& first, const Polygon2d& second, double eps)
{
    (void)eps;
    return BooleanCompose(first, second, BooleanOp::Difference);
}
} // namespace geometry::sdk
