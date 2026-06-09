#pragma once

#include "Export/GeometryExport.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCMultiPolygon2d.h"
#include "Geometry2d/SCMultiPolyline2d.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Types/Geometry2d/SCBox2.h"
#include "Types/Geometry2d/SCPoint2.h"
#include "Types/Geometry2d/SCVector2.h"

namespace Geometry
{
    [[nodiscard]] GEOMETRY_API SCPoint2d Translate(const SCPoint2d& point, const SCVector2d& offset);
    [[nodiscard]] GEOMETRY_API SCLineSegment2d Translate(const SCLineSegment2d& segment, const SCVector2d& offset);
    [[nodiscard]] GEOMETRY_API SCArcSegment2d Translate(const SCArcSegment2d& segment, const SCVector2d& offset);
    [[nodiscard]] GEOMETRY_API SCPolyline2d Translate(const SCPolyline2d& polyline, const SCVector2d& offset);
    [[nodiscard]] GEOMETRY_API SCPolygon2d Translate(const SCPolygon2d& polygon, const SCVector2d& offset);
    [[nodiscard]] GEOMETRY_API SCMultiPolyline2d Translate(const SCMultiPolyline2d& polylines, const SCVector2d& offset);
    [[nodiscard]] GEOMETRY_API SCMultiPolygon2d Translate(const SCMultiPolygon2d& polygons, const SCVector2d& offset);

    [[nodiscard]] GEOMETRY_API SCPoint2d Rotate(const SCPoint2d& point, const SCPoint2d& origin, double angleRadians);
    [[nodiscard]] GEOMETRY_API SCLineSegment2d Rotate(const SCLineSegment2d& segment,
                                                    const SCPoint2d& origin,
                                                    double angleRadians);
    [[nodiscard]] GEOMETRY_API SCArcSegment2d Rotate(const SCArcSegment2d& segment,
                                                   const SCPoint2d& origin,
                                                   double angleRadians);
    [[nodiscard]] GEOMETRY_API SCPolyline2d Rotate(const SCPolyline2d& polyline,
                                                 const SCPoint2d& origin,
                                                 double angleRadians);
    [[nodiscard]] GEOMETRY_API SCPolygon2d Rotate(const SCPolygon2d& polygon, const SCPoint2d& origin, double angleRadians);

    [[nodiscard]] GEOMETRY_API SCPoint2d Mirror(const SCPoint2d& point, const SCPoint2d& linePoint, const SCVector2d& lineDir);
    [[nodiscard]] GEOMETRY_API SCLineSegment2d Mirror(const SCLineSegment2d& segment,
                                                    const SCPoint2d& linePoint,
                                                    const SCVector2d& lineDir);
    [[nodiscard]] GEOMETRY_API SCArcSegment2d Mirror(const SCArcSegment2d& segment,
                                                   const SCPoint2d& linePoint,
                                                   const SCVector2d& lineDir);
    [[nodiscard]] GEOMETRY_API SCPolyline2d Mirror(const SCPolyline2d& polyline,
                                                 const SCPoint2d& linePoint,
                                                 const SCVector2d& lineDir);
    [[nodiscard]] GEOMETRY_API SCPolygon2d Mirror(const SCPolygon2d& polygon,
                                                const SCPoint2d& linePoint,
                                                const SCVector2d& lineDir);

    [[nodiscard]] GEOMETRY_API SCPoint2d Stretch(const SCPoint2d& point, const SCBox2d& region, const SCVector2d& offset);
    [[nodiscard]] GEOMETRY_API SCLineSegment2d Stretch(const SCLineSegment2d& segment,
                                                     const SCBox2d& region,
                                                     const SCVector2d& offset);
    [[nodiscard]] GEOMETRY_API SCArcSegment2d Stretch(const SCArcSegment2d& segment,
                                                    const SCBox2d& region,
                                                    const SCVector2d& offset);
    [[nodiscard]] GEOMETRY_API SCPolyline2d Stretch(const SCPolyline2d& polyline,
                                                  const SCBox2d& region,
                                                  const SCVector2d& offset);
    [[nodiscard]] GEOMETRY_API SCPolygon2d Stretch(const SCPolygon2d& polygon, const SCBox2d& region, const SCVector2d& offset);
}  // namespace Geometry
