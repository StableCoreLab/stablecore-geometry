#pragma once
#include "Export/GeometryExport.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Geometry3d/TriangleMesh.h"

namespace Geometry
{
    [[nodiscard]] GEOMETRY_API TriangleMesh Tessellate(const SCPlaneSurface& surface,
                                                       std::size_t uSegments = 1,
                                                       std::size_t vSegments = 1);
}  // namespace Geometry
