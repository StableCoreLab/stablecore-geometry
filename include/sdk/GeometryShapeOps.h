#pragma once

#include <string>

#include "export/GeometryExport.h"
#include "sdk/ArcSegment2d.h"
#include "sdk/LineSegment2d.h"
#include "sdk/Polyline2d.h"
#include "sdk/Polygon2d.h"
#include "sdk/GeometryResults.h"
#include "sdk/Segment2d.h"

namespace geometry::sdk
{
[[nodiscard]] GEOMETRY_API double Perimeter(const Polygon2d& polygon);
[[nodiscard]] GEOMETRY_API RingOrientation2d Orientation(const Polyline2d& ring);
[[nodiscard]] GEOMETRY_API bool IsClockwise(const Polyline2d& ring);
[[nodiscard]] GEOMETRY_API bool IsCounterClockwise(const Polyline2d& ring);
[[nodiscard]] GEOMETRY_API Point2d Centroid(const Polygon2d& polygon);

[[nodiscard]] GEOMETRY_API LineSegment2d Reverse(const LineSegment2d& segment);
[[nodiscard]] GEOMETRY_API ArcSegment2d Reverse(const ArcSegment2d& segment);
[[nodiscard]] GEOMETRY_API Polyline2d Reverse(const Polyline2d& polyline);
[[nodiscard]] GEOMETRY_API Polyline2d Close(const Polyline2d& polyline);
} // namespace geometry::sdk
