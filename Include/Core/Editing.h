#pragma once

#include "Export/GeometryExport.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    [[nodiscard]] GEOMETRY_API SCPolyline2d Normalize(const SCPolyline2d& polyline, double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCPolygon2d Normalize(const SCPolygon2d& polygon, double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCPolyline2d InsertPoint(const SCPolyline2d& polyline,
                                                      const SCPoint2d& point,
                                                      double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCPolygon2d InsertPoint(const SCPolygon2d& polygon,
                                                     const SCPoint2d& point,
                                                     double eps = Geometry::kDefaultEpsilon);
}  // namespace Geometry
