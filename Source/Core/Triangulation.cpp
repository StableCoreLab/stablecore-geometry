#include "Core/Triangulation.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "Core/Intersection.h"
#include "Core/Relation.h"
#include "Support/Epsilon.h"
#include "Support/Geometry2d/Predicate2.h"

namespace Geometry
{
    namespace
    {
        struct Node { SCPoint2d point{}; std::size_t index{0}; };

        [[nodiscard]] bool InTriangle(const SCPoint2d& p, const SCPoint2d& a, const SCPoint2d& b, const SCPoint2d& c)
        {
            const double x = Cross(b - a, p - a), y = Cross(c - b, p - b), z = Cross(a - c, p - c);
            const bool negative = x < -kDefaultEpsilon || y < -kDefaultEpsilon || z < -kDefaultEpsilon;
            const bool positive = x > kDefaultEpsilon || y > kDefaultEpsilon || z > kDefaultEpsilon;
            return !(negative && positive);
        }

        [[nodiscard]] bool AllowedIntersection(const SCSegmentIntersection2d& hit, const SCPoint2d& a, const SCPoint2d& b)
        {
            if (!hit.HasIntersection()) return true;
            if (hit.kind == SCIntersectionKind2d::Overlap) return false;
            for (std::size_t i = 0; i < hit.pointCount; ++i)
                if (!hit.points[i].point.AlmostEquals(a, kDefaultEpsilon) && !hit.points[i].point.AlmostEquals(b, kDefaultEpsilon)) return false;
            return true;
        }

        [[nodiscard]] bool Visible(const SCPoint2d& holePoint, const SCPoint2d& outerPoint,
                                   const std::vector<Node>& contour, const SCPolygon2d& polygon)
        {
            const SCLineSegment2d bridge(holePoint, outerPoint);
            if (!bridge.IsValid()) return false;
            for (const double t : {0.25, 0.5, 0.75})
                if (LocatePoint(holePoint + (outerPoint - holePoint) * t, polygon, kDefaultEpsilon) == SCPointContainment2d::Outside) return false;
            for (std::size_t i = 0; i < contour.size(); ++i)
            {
                const SCLineSegment2d edge(contour[i].point, contour[(i + 1) % contour.size()].point);
                if (!AllowedIntersection(Intersect(bridge, edge, kDefaultEpsilon), holePoint, outerPoint)) return false;
            }
            for (std::size_t h = 0; h < polygon.HoleCount(); ++h)
            {
                const SCPolyline2d hole = polygon.HoleAt(h);
                for (std::size_t i = 0; i < hole.VertexCount(); ++i)
                {
                    const SCLineSegment2d edge(hole.VertexAt(i), hole.VertexAt((i + 1) % hole.VertexCount()));
                    if (!AllowedIntersection(Intersect(bridge, edge, kDefaultEpsilon), holePoint, outerPoint)) return false;
                }
            }
            return true;
        }

        [[nodiscard]] std::size_t Rightmost(const std::vector<Node>& hole)
        {
            std::size_t result = 0;
            for (std::size_t i = 1; i < hole.size(); ++i)
                if (hole[i].point.x > hole[result].point.x ||
                    (std::abs(hole[i].point.x - hole[result].point.x) <= kDefaultEpsilon && hole[i].point.y < hole[result].point.y)) result = i;
            return result;
        }

        [[nodiscard]] bool MergeHole(const SCPolygon2d& polygon, const std::vector<Node>& hole, std::vector<Node>& contour)
        {
            const std::size_t h = Rightmost(hole);
            std::size_t outer = contour.size();
            double best = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < contour.size(); ++i)
            {
                const double distance = (contour[i].point - hole[h].point).LengthSquared();
                if (distance <= kDefaultEpsilon * kDefaultEpsilon || distance >= best || !Visible(hole[h].point, contour[i].point, contour, polygon)) continue;
                best = distance; outer = i;
            }
            if (outer == contour.size()) return false;
            std::vector<Node> merged;
            merged.reserve(contour.size() + hole.size() + 2);
            for (std::size_t i = 0; i <= outer; ++i) merged.push_back(contour[i]);
            merged.push_back(hole[h]);
            for (std::size_t step = 1; step < hole.size(); ++step) merged.push_back(hole[(h + step) % hole.size()]);
            merged.push_back(hole[h]);
            merged.push_back(contour[outer]);
            for (std::size_t i = outer + 1; i < contour.size(); ++i) merged.push_back(contour[i]);
            contour = std::move(merged);
            return true;
        }

        [[nodiscard]] bool IsEar(const std::vector<std::size_t>& polygon, std::size_t position, const std::vector<Node>& nodes)
        {
            const std::size_t count = polygon.size();
            const std::size_t p = polygon[(position + count - 1) % count], c = polygon[position], n = polygon[(position + 1) % count];
            const SCPoint2d& a = nodes[p].point, &b = nodes[c].point, &d = nodes[n].point;
            if (a.AlmostEquals(b, kDefaultEpsilon) || b.AlmostEquals(d, kDefaultEpsilon) || a.AlmostEquals(d, kDefaultEpsilon) || Cross(b - a, d - b) <= kDefaultEpsilon) return false;
            for (const std::size_t candidate : polygon)
            {
                if (candidate == p || candidate == c || candidate == n || nodes[candidate].index == nodes[p].index || nodes[candidate].index == nodes[c].index || nodes[candidate].index == nodes[n].index) continue;
                if (InTriangle(nodes[candidate].point, a, b, d)) return false;
            }
            return true;
        }
    }

    std::optional<std::vector<SCTriangleIndex3>> Triangulate(const SCPolygon2d& polygon)
    {
        if (!polygon.IsValid()) return std::nullopt;
        std::vector<Node> contour;
        std::size_t index = 0;
        const SCPolyline2d outer = polygon.OuterRing();
        contour.reserve(polygon.PointCount() + 2 * polygon.HoleCount());
        for (std::size_t i = 0; i < outer.VertexCount(); ++i) contour.push_back({outer.VertexAt(i), index++});
        for (std::size_t h = 0; h < polygon.HoleCount(); ++h)
        {
            const SCPolyline2d ring = polygon.HoleAt(h);
            std::vector<Node> hole;
            for (std::size_t i = 0; i < ring.VertexCount(); ++i) hole.push_back({ring.VertexAt(i), index++});
            if (!MergeHole(polygon, hole, contour)) return std::nullopt;
        }
        std::vector<std::size_t> remaining(contour.size());
        for (std::size_t i = 0; i < remaining.size(); ++i) remaining[i] = i;
        std::vector<SCTriangleIndex3> result;
        result.reserve(contour.size() - 2);
        while (remaining.size() > 3)
        {
            bool clipped = false;
            for (std::size_t i = 0; i < remaining.size(); ++i)
            {
                if (!IsEar(remaining, i, contour)) continue;
                const std::size_t p = remaining[(i + remaining.size() - 1) % remaining.size()], c = remaining[i], n = remaining[(i + 1) % remaining.size()];
                const SCTriangleIndex3 triangle{{contour[p].index, contour[c].index, contour[n].index}};
                if (triangle.indices[0] == triangle.indices[1] || triangle.indices[1] == triangle.indices[2] || triangle.indices[0] == triangle.indices[2]) return std::nullopt;
                result.push_back(triangle);
                remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(i)); clipped = true; break;
            }
            if (!clipped) return std::nullopt;
        }
        if (remaining.size() != 3) return std::nullopt;
        const SCTriangleIndex3 last{{contour[remaining[0]].index, contour[remaining[1]].index, contour[remaining[2]].index}};
        if (last.indices[0] == last.indices[1] || last.indices[1] == last.indices[2] || last.indices[0] == last.indices[2] ||
            Cross(contour[remaining[1]].point - contour[remaining[0]].point, contour[remaining[2]].point - contour[remaining[1]].point) <= kDefaultEpsilon) return std::nullopt;
        result.push_back(last);
        return result.size() == contour.size() - 2 ? std::optional<std::vector<SCTriangleIndex3>>(std::move(result)) : std::nullopt;
    }
}  // namespace Geometry
