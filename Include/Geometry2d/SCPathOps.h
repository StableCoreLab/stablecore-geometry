#pragma once

#include "Export/GeometryExport.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCMultiPolygon2d.h"
#include "Geometry2d/SCMultiPolyline2d.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    struct GEOMETRY_API SCPolygonCutResult2d
    {
        bool success{false};
        SCMultiPolygon2d left{};
        SCMultiPolygon2d right{};
    };

    [[nodiscard]] GEOMETRY_API SCPolyline2d SubPolyline(const SCPolyline2d& polyline, double startLength, double endLength);

    [[nodiscard]] GEOMETRY_API SCPolygonCutResult2d CutPolygon(const SCPolygon2d& polygon,
                                                               const SCLineSegment2d& cutter,
                                                               double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCMultiPolygon2d BuildMultiPolygonByLines(const SCMultiPolyline2d& polylines,
                                                                       double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCPolygon2d NormalizePolygonByLines(const SCPolygon2d& polygon,
                                                                 double eps = Geometry::kDefaultEpsilon);
}  // namespace Geometry
