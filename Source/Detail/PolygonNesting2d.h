#pragma once

#include <cstddef>
#include <limits>
#include <vector>

#include "Brep/Topology.h"

namespace Geometry::Detail
{
    inline constexpr std::size_t kNoContainingPolygon2d = std::numeric_limits<std::size_t>::max();

    [[nodiscard]] inline std::vector<std::size_t> BuildPolygonParents2d(const std::vector<SCPolygon2d>& polygons,
                                                                        const double tolerance)
    {
        std::vector<std::size_t> parents(polygons.size(), kNoContainingPolygon2d);
        for (std::size_t index = 0; index < polygons.size(); ++index)
        {
            const double polygonArea = polygons[index].Area();
            double smallestContainerArea = 0.0;
            for (std::size_t candidate = 0; candidate < polygons.size(); ++candidate)
            {
                if (index == candidate)
                {
                    continue;
                }

                const double candidateArea = polygons[candidate].Area();
                if (candidateArea <= polygonArea + tolerance ||
                    !Contains(polygons[candidate], polygons[index], tolerance))
                {
                    continue;
                }

                if (parents[index] == kNoContainingPolygon2d || candidateArea < smallestContainerArea)
                {
                    parents[index] = candidate;
                    smallestContainerArea = candidateArea;
                }
            }
        }

        return parents;
    }
}  // namespace Geometry::Detail
