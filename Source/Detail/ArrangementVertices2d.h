#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "Types/Geometry2d/SCPoint2.h"

namespace Geometry::Detail
{
    inline constexpr std::size_t kInvalidVertexIndex2d = std::numeric_limits<std::size_t>::max();

    [[nodiscard]] inline std::size_t FindVertexIndex2d(const std::vector<SCPoint2d>& vertices,
                                                       const SCPoint2d& point,
                                                       const double tolerance)
    {
        for (std::size_t index = 0; index < vertices.size(); ++index)
        {
            if (vertices[index].AlmostEquals(point, tolerance))
            {
                return index;
            }
        }
        return kInvalidVertexIndex2d;
    }

    [[nodiscard]] inline std::size_t FindOrAddVertex2d(std::vector<SCPoint2d>& vertices,
                                                       const SCPoint2d& point,
                                                       const double tolerance)
    {
        const std::size_t existing = FindVertexIndex2d(vertices, point, tolerance);
        if (existing != kInvalidVertexIndex2d)
        {
            return existing;
        }

        vertices.push_back(point);
        return vertices.size() - 1;
    }

    [[nodiscard]] inline std::uint64_t MakeUndirectedEdgeKey2d(const std::size_t first, const std::size_t second)
    {
        const std::uint64_t lower = static_cast<std::uint64_t>(std::min(first, second));
        const std::uint64_t upper = static_cast<std::uint64_t>(std::max(first, second));
        return (lower << 32U) | upper;
    }
}  // namespace Geometry::Detail
