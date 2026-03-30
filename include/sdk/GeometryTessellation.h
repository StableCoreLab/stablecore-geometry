#pragma once

#include <cstddef>

#include "export/GeometryExport.h"
#include "sdk/PlaneSurface.h"
#include "sdk/TriangleMesh.h"

namespace geometry::sdk
{
[[nodiscard]] GEOMETRY_API TriangleMesh Tessellate(
    const PlaneSurface& surface,
    std::size_t uSegments = 1,
    std::size_t vSegments = 1);
} // namespace geometry::sdk
