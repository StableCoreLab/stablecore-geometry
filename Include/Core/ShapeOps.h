#pragma once

#include <string>

#include "Core/Results.h"
#include "Export/GeometryExport.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Geometry2d/ISCSegment2d.h"
#include "Types/Geometry2d/SCBox2.h"
#include "Types/Geometry2d/SCPoint2.h"
#include "Types/Geometry2d/SCVector2.h"

namespace Geometry
{
    [[nodiscard]] GEOMETRY_API double Perimeter(const SCPolygon2d& polygon);
    [[nodiscard]] GEOMETRY_API SCRingOrientation2d Orientation(const SCPolyline2d& ring);
    [[nodiscard]] GEOMETRY_API bool IsClockwise(const SCPolyline2d& ring);
    [[nodiscard]] GEOMETRY_API bool IsCounterClockwise(const SCPolyline2d& ring);
    [[nodiscard]] GEOMETRY_API SCPoint2d Centroid(const SCPolygon2d& polygon);

    [[nodiscard]] GEOMETRY_API SCLineSegment2d Reverse(const SCLineSegment2d& segment);
    [[nodiscard]] GEOMETRY_API SCArcSegment2d Reverse(const SCArcSegment2d& segment);
    [[nodiscard]] GEOMETRY_API SCPolyline2d Reverse(const SCPolyline2d& polyline);
    [[nodiscard]] GEOMETRY_API SCPolyline2d Close(const SCPolyline2d& polyline);
}  // namespace Geometry

