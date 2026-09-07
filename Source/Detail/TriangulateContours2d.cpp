#include "TriangulateContours2d.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "Core/Intersection.h"
#include "Core/Relation.h"

namespace Geometry::Detail
{
    namespace
    {
        struct Node
        {
            SCPoint2d point{};
            std::size_t index{0};
        };

        [[nodiscard]] double Area(const SCPoint2d& a, const SCPoint2d& b, const SCPoint2d& c)
        {
            const SCVector2d ab = b - a;
            const SCVector2d ac = c - a;
            return 0.5 * (ab.x * ac.y - ab.y * ac.x);
        }

        [[nodiscard]] bool InTriangle(const SCPoint2d& p,
                                      const SCPoint2d& a,
                                      const SCPoint2d& b,
                                      const SCPoint2d& c,
                                      double tolerance)
        {
            const double first = Area(a, b, p), second = Area(b, c, p), third = Area(c, a, p);
            const bool negative = first < -tolerance || second < -tolerance || third < -tolerance;
            const bool positive = first > tolerance || second > tolerance || third > tolerance;
            return !(negative && positive);
        }

        [[nodiscard]] bool Visible(const Node& hole,
                                   const Node& outer,
                                   const std::vector<Node>& contour,
                                   const std::vector<std::vector<SCPoint2d>>& loops,
                                   double tolerance)
        {
            const SCLineSegment2d bridge(hole.point, outer.point);
            if (!bridge.IsValid()) return false;
            std::vector<SCPolyline2d> rings;
            rings.reserve(loops.size());
            for (const auto& loop : loops) rings.emplace_back(loop, SCPolylineClosure::Closed);
            const SCPolygon2d polygon(rings.front(), std::vector<SCPolyline2d>(rings.begin() + 1, rings.end()));
            for (const double t : {0.25, 0.5, 0.75})
            {
                if (LocatePoint(hole.point + (outer.point - hole.point) * t, polygon, tolerance) ==
                    SCPointContainment2d::Outside) return false;
            }
            for (std::size_t i = 0; i < contour.size(); ++i)
            {
                const SCLineSegment2d edge(contour[i].point, contour[(i + 1) % contour.size()].point);
                const SCSegmentIntersection2d hit = Intersect(bridge, edge, tolerance);
                if (hit.kind == SCIntersectionKind2d::Overlap) return false;
                for (std::size_t k = 0; k < hit.pointCount; ++k)
                {
                    if (!hit.points[k].point.AlmostEquals(hole.point, tolerance) &&
                        !hit.points[k].point.AlmostEquals(outer.point, tolerance)) return false;
                }
            }
            for (const auto& loop : loops)
            {
                for (std::size_t i = 0; i < loop.size(); ++i)
                {
                    const SCLineSegment2d edge(loop[i], loop[(i + 1) % loop.size()]);
                    const SCSegmentIntersection2d hit = Intersect(bridge, edge, tolerance);
                    if (hit.kind == SCIntersectionKind2d::Overlap) return false;
                    for (std::size_t k = 0; k < hit.pointCount; ++k)
                    {
                        if (!hit.points[k].point.AlmostEquals(hole.point, tolerance) &&
                            !hit.points[k].point.AlmostEquals(outer.point, tolerance)) return false;
                    }
                }
            }
            return true;
        }

        [[nodiscard]] std::size_t Rightmost(const std::vector<Node>& hole, double tolerance)
        {
            std::size_t result = 0;
            for (std::size_t i = 1; i < hole.size(); ++i)
            {
                if (hole[i].point.x > hole[result].point.x + tolerance ||
                    (std::abs(hole[i].point.x - hole[result].point.x) <= tolerance &&
                     hole[i].point.y < hole[result].point.y)) result = i;
            }
            return result;
        }

        [[nodiscard]] bool MergeHole(const std::vector<std::vector<SCPoint2d>>& loops,
                                     const std::vector<Node>& hole,
                                     std::vector<Node>& contour,
                                     double tolerance)
        {
            const std::size_t h = Rightmost(hole, tolerance);
            std::size_t outer = contour.size();
            double best = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < contour.size(); ++i)
            {
                const double distance = (contour[i].point - hole[h].point).LengthSquared();
                if (distance <= tolerance * tolerance || distance >= best ||
                    !Visible(hole[h], contour[i], contour, loops, tolerance)) continue;
                best = distance;
                outer = i;
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
    }  // namespace

    std::optional<std::vector<SCTriangleIndex3>> TriangulateContours2d(
        const std::vector<std::vector<SCPoint2d>>& loops, double tolerance)
    {
        if (loops.empty() || loops.front().size() < 3 || !std::isfinite(tolerance) || tolerance <= 0.0)
            return std::nullopt;
        std::vector<Node> contour;
        std::size_t index = 0;
        for (const auto& point : loops.front()) contour.push_back({point, index++});
        for (std::size_t h = 1; h < loops.size(); ++h)
        {
            if (loops[h].size() < 3) return std::nullopt;
            std::vector<Node> hole;
            for (const auto& point : loops[h]) hole.push_back({point, index++});
            if (!MergeHole(loops, hole, contour, tolerance)) return std::nullopt;
        }

        std::vector<std::size_t> remaining(contour.size());
        for (std::size_t i = 0; i < remaining.size(); ++i) remaining[i] = i;
        std::vector<SCTriangleIndex3> result;
        while (remaining.size() > 3)
        {
            bool clipped = false;
            for (std::size_t i = 0; i < remaining.size(); ++i)
            {
                const std::size_t previous = remaining[(i + remaining.size() - 1) % remaining.size()];
                const std::size_t current = remaining[i];
                const std::size_t next = remaining[(i + 1) % remaining.size()];
                const double area = Area(contour[previous].point, contour[current].point, contour[next].point);
                if (area <= tolerance) continue;
                bool contains = false;
                for (const std::size_t candidate : remaining)
                {
                    if (candidate == previous || candidate == current || candidate == next) continue;
                    if (contour[candidate].index == contour[previous].index ||
                        contour[candidate].index == contour[current].index ||
                        contour[candidate].index == contour[next].index) continue;
                    if (InTriangle(contour[candidate].point, contour[previous].point,
                                   contour[current].point, contour[next].point, tolerance))
                    {
                        contains = true;
                        break;
                    }
                }
                if (contains) continue;
                result.push_back({{contour[previous].index, contour[current].index, contour[next].index}});
                remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(i));
                clipped = true;
                break;
            }
            if (!clipped) return std::nullopt;
        }
        if (remaining.size() != 3 || Area(contour[remaining[0]].point, contour[remaining[1]].point,
                                          contour[remaining[2]].point) <= tolerance)
            return std::nullopt;
        result.push_back({{contour[remaining[0]].index, contour[remaining[1]].index, contour[remaining[2]].index}});
        return result;
    }
}  // namespace Geometry::Detail
