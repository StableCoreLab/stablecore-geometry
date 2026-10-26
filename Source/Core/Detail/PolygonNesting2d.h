#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "Core/PolygonRelation2d.h"
#include "Support/StableNumerics.h"

namespace Geometry::Detail
{
    inline constexpr std::size_t kNoContainingPolygon2d = std::numeric_limits<std::size_t>::max();

    [[nodiscard]] inline std::vector<std::size_t> BuildPolygonParents2d(const std::vector<SCPolygon2d>& polygons,
                                                                        const double tolerance)
    {
        std::vector<std::size_t> parents(polygons.size(), kNoContainingPolygon2d);
        for (std::size_t index = 0; index < polygons.size(); ++index)
        {
            const double polygonArea = polygons[index].Area(tolerance);
            double smallestContainerArea = 0.0;
            for (std::size_t candidate = 0; candidate < polygons.size(); ++candidate)
            {
                if (index == candidate)
                {
                    continue;
                }

                const double candidateArea = polygons[candidate].Area(tolerance);
                const SCBox2d bounds = polygons[candidate].Bounds(tolerance);
                const double scale = bounds.IsValid() ? std::hypot(bounds.Width(), bounds.Height()) : 0.0;
                const auto scaledTolerance = StableProduct(tolerance, scale, 2);
                const auto absoluteTolerance = StableProduct(tolerance, tolerance, 4);
                if (!scaledTolerance || !absoluteTolerance || candidateArea <= polygonArea ||
                    candidateArea - polygonArea <= std::max(*scaledTolerance, *absoluteTolerance) ||
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
