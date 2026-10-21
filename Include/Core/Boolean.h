#pragma once

#include "Export/GeometryExport.h"
#include "Geometry2d/SCMultiPolygon2d.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    // 以下三个布尔入口的 eps 均须为正且有限；无效容差返回空结果。
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
