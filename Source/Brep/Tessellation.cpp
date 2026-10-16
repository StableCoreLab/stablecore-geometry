#include "Brep/Tessellation.h"

#include <algorithm>
#include <vector>
#include "Support/StableNumerics.h"

namespace Geometry
{
    TriangleMesh Tessellate(const SCPlaneSurface& surface, std::size_t uSegments, std::size_t vSegments)
    {
        if (!surface.IsValid())
        {
            return {};
        }

        uSegments = std::max<std::size_t>(uSegments, 1);
        vSegments = std::max<std::size_t>(vSegments, 1);

        const SCIntervald uRange = surface.URange();
        const SCIntervald vRange = surface.VRange();
        if (!uRange.IsValid() || !vRange.IsValid())
        {
            return {};
        }

        std::vector<SCPoint3d> vertices;
        std::vector<TriangleMesh::TriangleIndices> triangles;
        vertices.reserve((uSegments + 1) * (vSegments + 1));
        triangles.reserve(uSegments * vSegments * 2);

        for (std::size_t v = 0; v <= vSegments; ++v)
        {
            const double vRatio = static_cast<double>(v) / static_cast<double>(vSegments);
            const auto vParameter = Detail::StableLerp(vRange.min, vRange.max, vRatio);
            if (!vParameter) return {};
            for (std::size_t u = 0; u <= uSegments; ++u)
            {
                const double uRatio = static_cast<double>(u) / static_cast<double>(uSegments);
                const auto uParameter = Detail::StableLerp(uRange.min, uRange.max, uRatio);
                if (!uParameter) return {};
                const auto point=surface.PointAt(*uParameter, *vParameter);
                if (!point.IsValid()) return {};
                vertices.push_back(point);
            }
        }

        const auto vertexIndex = [uSegments](std::size_t u, std::size_t v) { return v * (uSegments + 1) + u; };

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
}  // namespace Geometry
