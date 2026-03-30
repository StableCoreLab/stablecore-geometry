#include "sdk/GeometryTessellation.h"

#include <algorithm>
#include <vector>

namespace geometry::sdk
{
TriangleMesh Tessellate(const PlaneSurface& surface, std::size_t uSegments, std::size_t vSegments)
{
    if (!surface.IsValid())
    {
        return {};
    }

    uSegments = std::max<std::size_t>(uSegments, 1);
    vSegments = std::max<std::size_t>(vSegments, 1);

    const Intervald uRange = surface.URange();
    const Intervald vRange = surface.VRange();
    if (!uRange.IsValid() || !vRange.IsValid())
    {
        return {};
    }

    std::vector<Point3d> vertices;
    std::vector<TriangleMesh::TriangleIndices> triangles;
    vertices.reserve((uSegments + 1) * (vSegments + 1));
    triangles.reserve(uSegments * vSegments * 2);

    for (std::size_t v = 0; v <= vSegments; ++v)
    {
        const double vRatio = static_cast<double>(v) / static_cast<double>(vSegments);
        const double vValue = vRange.min + vRange.Length() * vRatio;
        for (std::size_t u = 0; u <= uSegments; ++u)
        {
            const double uRatio = static_cast<double>(u) / static_cast<double>(uSegments);
            const double uValue = uRange.min + uRange.Length() * uRatio;
            vertices.push_back(surface.PointAt(uValue, vValue));
        }
    }

    const auto vertexIndex = [uSegments](std::size_t u, std::size_t v) {
        return v * (uSegments + 1) + u;
    };

    for (std::size_t v = 0; v < vSegments; ++v)
    {
        for (std::size_t u = 0; u < uSegments; ++u)
        {
            const std::size_t i00 = vertexIndex(u, v);
            const std::size_t i10 = vertexIndex(u + 1, v);
            const std::size_t i01 = vertexIndex(u, v + 1);
            const std::size_t i11 = vertexIndex(u + 1, v + 1);
            triangles.push_back(TriangleMesh::TriangleIndices{i00, i10, i11});
            triangles.push_back(TriangleMesh::TriangleIndices{i00, i11, i01});
        }
    }

    return TriangleMesh(std::move(vertices), std::move(triangles));
}
} // namespace geometry::sdk
