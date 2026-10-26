#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "Core/Triangulation.h"
#include "Types/Geometry2d/SCPoint2.h"

namespace Geometry::Detail
{
    [[nodiscard]] std::optional<std::vector<SCTriangleIndex3>> TriangulateContours2d(
        const std::vector<std::vector<SCPoint2d>>& loops, double tolerance);
}  // namespace Geometry::Detail
