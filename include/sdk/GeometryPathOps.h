#pragma once

#include "export/GeometryExport.h"
#include "sdk/LineSegment2d.h"
#include "sdk/MultiPolygon2d.h"
#include "sdk/MultiPolyline2d.h"
#include "sdk/Polygon2d.h"
#include "sdk/Polyline2d.h"

namespace geometry::sdk
{
struct GEOMETRY_API PolygonCutResult2d
{
    bool success{false};
    MultiPolygon2d left{};
    MultiPolygon2d right{};
};

[[nodiscard]] GEOMETRY_API Polyline2d SubPolyline(
    const Polyline2d& polyline,
    double startLength,
    double endLength);

[[nodiscard]] GEOMETRY_API PolygonCutResult2d CutPolygon(
    const Polygon2d& polygon,
    const LineSegment2d& cutter,
    double eps = 1e-9);

[[nodiscard]] GEOMETRY_API MultiPolygon2d BuildMultiPolygonByLines(
    const MultiPolyline2d& polylines,
    double eps = 1e-9);
} // namespace geometry::sdk
