#pragma once

#include "Geometry2d/SCMultiPolygon2d.h"

namespace Geometry::Test
{
    [[nodiscard]] inline double TotalArea(const SCMultiPolygon2d& polygons)
    {
        double total = 0.0;
        for (std::size_t index = 0; index < polygons.Size(); ++index)
        {
            total += polygons[index].Area();
        }
        return total;
    }
}  // namespace Geometry::Test
