#pragma once

#include <array>
#include <optional>
#include <vector>

#include "Export/GeometryExport.h"
#include "Geometry2d/SCPolygon2d.h"

namespace Geometry
{
    struct GEOMETRY_API SCTriangleIndex3
    {
        std::array<std::size_t, 3> indices{};

        [[nodiscard]] constexpr bool operator==(const SCTriangleIndex3& other) const = default;
        [[nodiscard]] constexpr bool operator!=(const SCTriangleIndex3& other) const = default;
    };

    // For a polygon without holes, indices refer to OuterRing() vertices.
    // With holes, indices refer to flattened boundary vertices: outer ring
    // first, followed by HoleAt(0), HoleAt(1), ... .
    [[nodiscard]] GEOMETRY_API std::optional<std::vector<SCTriangleIndex3>>
    Triangulate(const SCPolygon2d& polygon);
}  // namespace Geometry
