#pragma once

#include <array>
#include <cstddef>

namespace Geometry::Detail
{
    // 与三角形顶点次序一致；边下标及方向供邻接、边界和定向修复共同使用。
    [[nodiscard]] constexpr std::array<std::array<std::size_t, 2>, 3> TriangleEdges3d(
        const std::array<std::size_t, 3>& tri)
    {
        return {{{tri[0], tri[1]}, {tri[1], tri[2]}, {tri[2], tri[0]}}};
    }
}
