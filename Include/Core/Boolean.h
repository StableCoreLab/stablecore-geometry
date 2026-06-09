#pragma once

#include "Export/GeometryExport.h"
#include "Geometry2d/SCMultiPolygon2d.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    [[nodiscard]] GEOMETRY_API SCMultiPolygon2d Intersect(const SCPolygon2d& first,
                                                        const SCPolygon2d& second,
                                                        double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCMultiPolygon2d Union(const SCPolygon2d& first,
                                                    const SCPolygon2d& second,
                                                    double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCMultiPolygon2d Difference(const SCPolygon2d& first,
                                                         const SCPolygon2d& second,
                                                         double eps = Geometry::kDefaultEpsilon);
}  // namespace Geometry
