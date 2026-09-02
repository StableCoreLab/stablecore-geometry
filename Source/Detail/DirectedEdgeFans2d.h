#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <vector>

namespace Geometry::Detail
{
    struct DirectedEdge2d
    {
        std::size_t from{0};
        std::size_t to{0};
        std::size_t twin{0};
        double angle{0.0};
    };

    inline constexpr std::size_t kInvalidDirectedEdge2d = std::numeric_limits<std::size_t>::max();

    template <typename TDirectedEdge>
    void SortOutgoingFans(const std::vector<TDirectedEdge>& edges, std::vector<std::vector<std::size_t>>& outgoing)
    {
        for (std::vector<std::size_t>& fan : outgoing)
        {
            std::sort(fan.begin(), fan.end(), [&edges](const std::size_t lhs, const std::size_t rhs) {
                if (edges[lhs].angle != edges[rhs].angle)
                {
                    return edges[lhs].angle < edges[rhs].angle;
                }
                if (edges[lhs].to != edges[rhs].to)
                {
                    return edges[lhs].to < edges[rhs].to;
                }
                return lhs < rhs;
            });
        }
    }

    [[nodiscard]] inline std::size_t PreviousOutgoing(const std::vector<std::size_t>& fan, const std::size_t edgeIndex)
    {
        for (std::size_t index = 0; index < fan.size(); ++index)
        {
            if (fan[index] == edgeIndex)
            {
                return fan[(index + fan.size() - 1) % fan.size()];
            }
        }
        return kInvalidDirectedEdge2d;
    }

    template <typename TDirectedEdge>
    [[nodiscard]] std::size_t NextFaceEdge(const std::vector<TDirectedEdge>& edges,
                                           const std::vector<std::vector<std::size_t>>& outgoing,
                                           const std::size_t edgeIndex)
    {
        if (edgeIndex >= edges.size())
        {
            return kInvalidDirectedEdge2d;
        }

        const TDirectedEdge& edge = edges[edgeIndex];
        if (edge.to >= outgoing.size())
        {
            return kInvalidDirectedEdge2d;
        }

        const std::vector<std::size_t>& fan = outgoing[edge.to];
        if (fan.empty())
        {
            return kInvalidDirectedEdge2d;
        }
        return PreviousOutgoing(fan, edge.twin);
    }
}  // namespace Geometry::Detail
