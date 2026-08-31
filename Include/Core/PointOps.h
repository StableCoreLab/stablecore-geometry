#pragma once

#include "Export/GeometryExport.h"
#include "Types/Geometry2d/SCPoint2.h"
#include "Types/Geometry3d/SCPoint3.h"

namespace Geometry
{
    [[nodiscard]] GEOMETRY_API SCPoint2d Midpoint(const SCPoint2d& first, const SCPoint2d& second) noexcept;

    [[nodiscard]] GEOMETRY_API SCPoint3d Midpoint(const SCPoint3d& first, const SCPoint3d& second) noexcept;
}  // namespace Geometry
