#include "TriangulateContours2d.h"
#include "PolylineRing2d.h"

#include <cmath>
#include <utility>

#include "Geometry2d/SCPolygon2d.h"

namespace Geometry::Detail
{
    std::optional<std::vector<SCTriangleIndex3>> TriangulateContours2d(
        const std::vector<std::vector<SCPoint2d>>& loops, double tolerance)
    {
        if (loops.empty() || !std::isfinite(tolerance) || tolerance < 0.0)
        {
            return std::nullopt;
        }
        std::vector<SCPolyline2d> holes;
        holes.reserve(loops.size() - 1);
        for (std::size_t i = 1; i < loops.size(); ++i)
        {
            holes.push_back(MakeVertexPreservingRing(loops[i]));
        }
        return Triangulate(SCPolygon2d(
            MakeVertexPreservingRing(loops.front()), std::move(holes)), tolerance);
    }
}  // namespace Geometry::Detail
