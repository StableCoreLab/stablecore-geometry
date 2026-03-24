#pragma once

#include "export/GeometryExport.h"
#include "sdk/ArcSegment2d.h"
#include "sdk/LineSegment2d.h"
#include "sdk/MultiPolygon2d.h"
#include "sdk/MultiPolyline2d.h"
#include "sdk/Polygon2d.h"
#include "sdk/Polyline2d.h"

namespace geometry::sdk
{
struct GEOMETRY_API OffsetOptions2d
{
    double miterLimit{4.0};
};

[[nodiscard]] GEOMETRY_API LineSegment2d Offset(const LineSegment2d& segment, double distance);
[[nodiscard]] GEOMETRY_API ArcSegment2d Offset(const ArcSegment2d& segment, double distance);
[[nodiscard]] GEOMETRY_API Polyline2d Offset(
    const Polyline2d& polyline,
    double distance,
    OffsetOptions2d options = {});
[[nodiscard]] GEOMETRY_API Polygon2d Offset(
    const Polygon2d& polygon,
    double distance,
    OffsetOptions2d options = {});
[[nodiscard]] GEOMETRY_API MultiPolyline2d Offset(
    const MultiPolyline2d& polylines,
    double distance,
    OffsetOptions2d options = {});
[[nodiscard]] GEOMETRY_API MultiPolygon2d Offset(
    const MultiPolygon2d& polygons,
    double distance,
    OffsetOptions2d options = {});
} // namespace geometry::sdk
