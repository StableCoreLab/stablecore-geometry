#include "Core/Triangulation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "Core/Relation.h"
#include "Support/Geometry2d/Predicate2.h"

namespace Geometry
{
    namespace
    {
        struct Node { SCPoint2d point{}; std::size_t index{0}; };

        [[nodiscard]] double SideDistance(const SCPoint2d& p, const SCPoint2d& a, const SCPoint2d& b)
        {
            const SCVector2d edge = b - a;
            return Cross(edge, p - a) / std::hypot(edge.x, edge.y);
        }

        [[nodiscard]] bool OnEdge(const SCPoint2d& p, const SCPoint2d& a, const SCPoint2d& b, double eps)
        {
            return std::abs(SideDistance(p, a, b)) <= eps &&
                   p.x >= std::min(a.x, b.x) - eps && p.x <= std::max(a.x, b.x) + eps &&
                   p.y >= std::min(a.y, b.y) - eps && p.y <= std::max(a.y, b.y) + eps;
        }

        enum class EdgeHit { None, Endpoint, Crossing, Overlap };

        [[nodiscard]] EdgeHit EdgeIntersection(const SCPoint2d& a, const SCPoint2d& b,
                                               const SCPoint2d& c, const SCPoint2d& d, double eps)
        {
            const double abC = SideDistance(c, a, b), abD = SideDistance(d, a, b);
            const double cdA = SideDistance(a, c, d), cdB = SideDistance(b, c, d);
            if (std::abs(abC) <= eps && std::abs(abD) <= eps)
            {
                const bool xAxis = std::abs(b.x - a.x) >= std::abs(b.y - a.y);
                const double a0 = xAxis ? a.x : a.y, b0 = xAxis ? b.x : b.y;
                const double c0 = xAxis ? c.x : c.y, d0 = xAxis ? d.x : d.y;
                const double overlap = std::min(std::max(a0, b0), std::max(c0, d0)) -
                                       std::max(std::min(a0, b0), std::min(c0, d0));
                if (overlap > eps) return EdgeHit::Overlap;
                return overlap >= -eps ? EdgeHit::Endpoint : EdgeHit::None;
            }
            if (((abC > eps && abD < -eps) || (abC < -eps && abD > eps)) &&
                ((cdA > eps && cdB < -eps) || (cdA < -eps && cdB > eps))) return EdgeHit::Crossing;
            return OnEdge(a, c, d, eps) || OnEdge(b, c, d, eps) ||
                   OnEdge(c, a, b, eps) || OnEdge(d, a, b, eps) ? EdgeHit::Endpoint : EdgeHit::None;
        }

        [[nodiscard]] SCPointContainment2d LocateInRing(const SCPoint2d& p, const SCPolyline2d& ring, double eps)
        {
            bool inside = false;
            for (std::size_t i = 0; i < ring.VertexCount(); ++i)
            {
                const SCPoint2d a = ring.VertexAt(i), b = ring.VertexAt((i + 1) % ring.VertexCount());
                if (OnEdge(p, a, b, eps)) return SCPointContainment2d::OnBoundary;
                if ((a.y > p.y) != (b.y > p.y) &&
                    a.x + (b.x - a.x) * (p.y - a.y) / (b.y - a.y) > p.x + eps) inside = !inside;
            }
            return inside ? SCPointContainment2d::Inside : SCPointContainment2d::Outside;
        }

        [[nodiscard]] double RingArea(const SCPolyline2d& ring)
        {
            double twiceArea = 0.0;
            for (std::size_t i = 0; i < ring.VertexCount(); ++i)
            {
                const SCPoint2d a = ring.VertexAt(i);
                const SCPoint2d b = ring.VertexAt((i + 1) % ring.VertexCount());
                twiceArea += a.x * b.y - b.x * a.y;
            }
            return twiceArea * 0.5;
        }

        [[nodiscard]] bool ValidRing(const SCPolyline2d& ring, double eps)
        {
            if (!ring.IsClosed() || ring.VertexCount() < 3 || ring.SegmentCount() != ring.VertexCount())
            {
                return false;
            }
            for (std::size_t i = 0; i < ring.VertexCount(); ++i)
            {
                const auto segment = ring.SegmentAt(i);
                if (!segment || segment->Kind() != SCSegmentKind2::Line) return false;
                const SCPoint2d a = segment->StartPoint();
                const auto next = ring.SegmentAt((i + 1) % ring.VertexCount());
                if (!next) return false;
                const SCPoint2d b = next->StartPoint();
                if (!a.IsValid() || !b.IsValid() || a.AlmostEquals(b, eps))
                {
                    return false;
                }
                if (!segment->EndPoint().AlmostEquals(b, eps))
                {
                    return false;
                }
            }
            if (std::abs(RingArea(ring)) <= eps * eps) return false;
            for (std::size_t i = 0; i < ring.VertexCount(); ++i)
            {
                if (EdgeIntersection(ring.VertexAt(i), ring.VertexAt((i + 1) % ring.VertexCount()),
                                     ring.VertexAt((i + 1) % ring.VertexCount()),
                                     ring.VertexAt((i + 2) % ring.VertexCount()), eps) == EdgeHit::Overlap)
                    return false;
                for (std::size_t j = i + 1; j < ring.VertexCount(); ++j)
                {
                    if (j == i + 1 || (i == 0 && j + 1 == ring.VertexCount())) continue;
                    if (EdgeIntersection(ring.VertexAt(i), ring.VertexAt((i + 1) % ring.VertexCount()),
                                         ring.VertexAt(j), ring.VertexAt((j + 1) % ring.VertexCount()), eps) != EdgeHit::None)
                        return false;
                }
            }
            return true;
        }

        [[nodiscard]] bool RingsIntersect(const SCPolyline2d& first, const SCPolyline2d& second, double eps)
        {
            for (std::size_t i = 0; i < first.SegmentCount(); ++i)
            {
                for (std::size_t j = 0; j < second.SegmentCount(); ++j)
                {
                    if (EdgeIntersection(first.VertexAt(i), first.VertexAt((i + 1) % first.VertexCount()),
                                         second.VertexAt(j), second.VertexAt((j + 1) % second.VertexCount()), eps) != EdgeHit::None)
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        [[nodiscard]] bool ValidBoundary(const SCPolygon2d& polygon, double eps)
        {
            const SCPolyline2d outer = polygon.OuterRing();
            if (!ValidRing(outer, eps))
            {
                return false;
            }
            std::vector<SCPolyline2d> holes;
            holes.reserve(polygon.HoleCount());
            for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
            {
                SCPolyline2d hole = polygon.HoleAt(i);
                if (!ValidRing(hole, eps) || RingsIntersect(outer, hole, eps) ||
                    LocateInRing(hole.VertexAt(0), outer, eps) != SCPointContainment2d::Inside)
                {
                    return false;
                }
                for (const SCPolyline2d& previous : holes)
                {
                    if (RingsIntersect(previous, hole, eps) ||
                        LocateInRing(hole.VertexAt(0), previous, eps) == SCPointContainment2d::Inside ||
                        LocateInRing(previous.VertexAt(0), hole, eps) == SCPointContainment2d::Inside)
                    {
                        return false;
                    }
                }
                holes.push_back(std::move(hole));
            }
            return true;
        }

        [[nodiscard]] SCPointContainment2d LocateInBoundary(const SCPoint2d& point,
                                                             const SCPolygon2d& polygon,
                                                             double eps)
        {
            const auto outer = LocateInRing(point, polygon.OuterRing(), eps);
            if (outer != SCPointContainment2d::Inside)
            {
                return outer;
            }
            for (std::size_t i = 0; i < polygon.HoleCount(); ++i)
            {
                const auto hole = LocateInRing(point, polygon.HoleAt(i), eps);
                if (hole == SCPointContainment2d::OnBoundary)
                {
                    return hole;
                }
                if (hole == SCPointContainment2d::Inside)
                {
                    return SCPointContainment2d::Outside;
                }
            }
            return SCPointContainment2d::Inside;
        }

        [[nodiscard]] bool InTriangle(const SCPoint2d& p, const SCPoint2d& a, const SCPoint2d& b, const SCPoint2d& c, double eps)
        {
            const double x = SideDistance(p, a, b), y = SideDistance(p, b, c),
                         z = SideDistance(p, c, a);
            const bool negative = x < -eps || y < -eps || z < -eps;
            const bool positive = x > eps || y > eps || z > eps;
            return !(negative && positive);
        }

        [[nodiscard]] bool AllowedIntersection(const SCPoint2d& a, const SCPoint2d& b,
                                               const SCPoint2d& c, const SCPoint2d& d, double eps)
        {
            const EdgeHit hit = EdgeIntersection(a, b, c, d, eps);
            if (hit == EdgeHit::None) return true;
            if (hit != EdgeHit::Endpoint) return false;
            return (OnEdge(a, c, d, eps) || OnEdge(b, c, d, eps)) &&
                   (!OnEdge(c, a, b, eps) || c.AlmostEquals(a, eps) || c.AlmostEquals(b, eps)) &&
                   (!OnEdge(d, a, b, eps) || d.AlmostEquals(a, eps) || d.AlmostEquals(b, eps));
        }

        [[nodiscard]] bool Visible(const SCPoint2d& holePoint, const SCPoint2d& outerPoint,
                                   const std::vector<Node>& contour, const SCPolygon2d& polygon, double eps)
        {
            if (holePoint.AlmostEquals(outerPoint, eps)) return false;
            for (const double t : {0.25, 0.5, 0.75})
                if (LocateInBoundary(holePoint + (outerPoint - holePoint) * t, polygon, eps) == SCPointContainment2d::Outside) return false;
            for (std::size_t i = 0; i < contour.size(); ++i)
            {
                if (!AllowedIntersection(holePoint, outerPoint, contour[i].point,
                                         contour[(i + 1) % contour.size()].point, eps)) return false;
            }
            for (std::size_t h = 0; h < polygon.HoleCount(); ++h)
            {
                const SCPolyline2d hole = polygon.HoleAt(h);
                for (std::size_t i = 0; i < hole.VertexCount(); ++i)
                {
                    if (!AllowedIntersection(holePoint, outerPoint, hole.VertexAt(i),
                                             hole.VertexAt((i + 1) % hole.VertexCount()), eps)) return false;
                }
            }
            return true;
        }

        [[nodiscard]] std::size_t Rightmost(const std::vector<Node>& hole, double eps)
        {
            std::size_t result = 0;
            for (std::size_t i = 1; i < hole.size(); ++i)
                if (hole[i].point.x > hole[result].point.x ||
                    (std::abs(hole[i].point.x - hole[result].point.x) <= eps && hole[i].point.y < hole[result].point.y)) result = i;
            return result;
        }

        [[nodiscard]] bool MergeHole(const SCPolygon2d& polygon, const std::vector<Node>& hole, std::vector<Node>& contour, double eps)
        {
            const std::size_t h = Rightmost(hole, eps);
            std::size_t outer = contour.size();
            double best = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < contour.size(); ++i)
            {
                const double distance = (contour[i].point - hole[h].point).LengthSquared();
                if (distance <= eps * eps || distance >= best || !Visible(hole[h].point, contour[i].point, contour, polygon, eps)) continue;
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

        [[nodiscard]] bool IsEar(const std::vector<std::size_t>& polygon, std::size_t position, const std::vector<Node>& nodes, double eps)
        {
            const std::size_t count = polygon.size();
            const std::size_t p = polygon[(position + count - 1) % count], c = polygon[position], n = polygon[(position + 1) % count];
            const SCPoint2d& a = nodes[p].point, &b = nodes[c].point, &d = nodes[n].point;
            if (a.AlmostEquals(b, eps) || b.AlmostEquals(d, eps) || a.AlmostEquals(d, eps) ||
                SideDistance(d, a, b) <= eps) return false;
            for (const std::size_t candidate : polygon)
            {
                if (candidate == p || candidate == c || candidate == n || nodes[candidate].index == nodes[p].index || nodes[candidate].index == nodes[c].index || nodes[candidate].index == nodes[n].index) continue;
                if (InTriangle(nodes[candidate].point, a, b, d, eps)) return false;
            }
            return true;
        }
    }

    std::optional<std::vector<SCTriangleIndex3>> Triangulate(const SCPolygon2d& polygon, double tolerance)
    {
        if (!std::isfinite(tolerance) || tolerance < 0.0 || !ValidBoundary(polygon, tolerance)) return std::nullopt;
        std::vector<Node> contour;
        std::size_t index = 0;
        const SCPolyline2d outer = polygon.OuterRing();
        contour.reserve(polygon.PointCount() + 2 * polygon.HoleCount());
        const bool outerReversed = RingArea(outer) < 0.0;
        for (std::size_t i = 0; i < outer.VertexCount(); ++i)
        {
            const std::size_t source = outerReversed ? outer.VertexCount() - 1 - i : i;
            contour.push_back({outer.VertexAt(source), source});
        }
        index = outer.VertexCount();
        for (std::size_t h = 0; h < polygon.HoleCount(); ++h)
        {
            const SCPolyline2d ring = polygon.HoleAt(h);
            std::vector<Node> hole;
            const bool holeReversed = RingArea(ring) > 0.0;
            for (std::size_t i = 0; i < ring.VertexCount(); ++i)
            {
                const std::size_t source = holeReversed ? ring.VertexCount() - 1 - i : i;
                hole.push_back({ring.VertexAt(source), index + source});
            }
            index += ring.VertexCount();
            if (!MergeHole(polygon, hole, contour, tolerance)) return std::nullopt;
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
                if (!IsEar(remaining, i, contour, tolerance)) continue;
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
            SideDistance(contour[remaining[2]].point, contour[remaining[0]].point,
                         contour[remaining[1]].point) <= tolerance) return std::nullopt;
        result.push_back(last);
        return result.size() == contour.size() - 2 ? std::optional<std::vector<SCTriangleIndex3>>(std::move(result)) : std::nullopt;
    }
}  // namespace Geometry
