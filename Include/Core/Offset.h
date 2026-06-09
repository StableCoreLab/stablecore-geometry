#pragma once

#include "Export/GeometryExport.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCMultiPolygon2d.h"
#include "Geometry2d/SCMultiPolyline2d.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Geometry2d/SCPolyline2d.h"

namespace Geometry
{
    struct GEOMETRY_API SCOffsetOptions2d
    {
        double miterLimit{4.0};
    };

    [[nodiscard]] GEOMETRY_API SCLineSegment2d Offset(const SCLineSegment2d& segment, double distance);
    [[nodiscard]] GEOMETRY_API SCArcSegment2d Offset(const SCArcSegment2d& segment, double distance);
    [[nodiscard]] GEOMETRY_API SCPolyline2d Offset(const SCPolyline2d& polyline,
                                                 double distance,
                                                 SCOffsetOptions2d options = {});
    [[nodiscard]] GEOMETRY_API SCPolygon2d Offset(const SCPolygon2d& polygon,
                                                double distance,
                                                SCOffsetOptions2d options = {});
    [[nodiscard]] GEOMETRY_API SCMultiPolygon2d OffsetToMultiPolygon(const SCPolygon2d& polygon,
                                                                   double distance,
                                                                   SCOffsetOptions2d options = {});
    [[nodiscard]] GEOMETRY_API SCMultiPolyline2d Offset(const SCMultiPolyline2d& polylines,
                                                      double distance,
                                                      SCOffsetOptions2d options = {});
    [[nodiscard]] GEOMETRY_API SCMultiPolygon2d Offset(const SCMultiPolygon2d& polygons,
                                                     double distance,
                                                     SCOffsetOptions2d options = {});
}  // namespace Geometry

