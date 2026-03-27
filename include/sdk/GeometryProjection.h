#pragma once

#include "export/GeometryExport.h"
#include "sdk/ArcSegment2d.h"
#include "sdk/GeometryTypes.h"
#include "sdk/LineSegment2d.h"
#include "sdk/Segment2d.h"

namespace geometry::sdk
{
[[nodiscard]] GEOMETRY_API SegmentProjection2d ProjectPointToLineSegment(
    const Point2d& point,
    const LineSegment2d& segment,
    bool clampToSegment = true);

[[nodiscard]] GEOMETRY_API SegmentProjection2d ProjectPointToArcSegment(
    const Point2d& point,
    const ArcSegment2d& segment,
    bool clampToSegment = true);

[[nodiscard]] GEOMETRY_API SegmentProjection2d ProjectPointToSegment(
    const Point2d& point,
    const Segment2d& segment,
    bool clampToSegment = true);

[[nodiscard]] GEOMETRY_API SegmentProjection2d ProjectPointToSegment(
    const Point2d& point,
    const Point2d& segmentStart,
    const Point2d& segmentEnd,
    bool clampToSegment = true);

[[nodiscard]] GEOMETRY_API LineProjection3d ProjectPointToLine(
    const Point3d& point,
    const Line3d& line,
    const GeometryTolerance3d& tolerance = {});

[[nodiscard]] GEOMETRY_API PlaneProjection3d ProjectPointToPlane(
    const Point3d& point,
    const Plane& plane,
    const GeometryTolerance3d& tolerance = {});

[[nodiscard]] GEOMETRY_API Point2d PointAt(const LineSegment2d& segment, double parameter);
[[nodiscard]] GEOMETRY_API Point2d PointAt(const ArcSegment2d& segment, double parameter);
[[nodiscard]] GEOMETRY_API Point2d PointAt(const Segment2d& segment, double parameter);

[[nodiscard]] GEOMETRY_API Point2d PointAtLength(
    const LineSegment2d& segment,
    double length,
    bool clampToSegment = false);
[[nodiscard]] GEOMETRY_API Point2d PointAtLength(
    const ArcSegment2d& segment,
    double length,
    bool clampToSegment = false);
[[nodiscard]] GEOMETRY_API Point2d PointAtLength(
    const Segment2d& segment,
    double length,
    bool clampToSegment = false);

[[nodiscard]] GEOMETRY_API double ParameterAtLength(
    const LineSegment2d& segment,
    double length,
    bool clampToSegment = false);
[[nodiscard]] GEOMETRY_API double ParameterAtLength(
    const ArcSegment2d& segment,
    double length,
    bool clampToSegment = false);
[[nodiscard]] GEOMETRY_API double ParameterAtLength(
    const Segment2d& segment,
    double length,
    bool clampToSegment = false);

[[nodiscard]] GEOMETRY_API Vector2d TangentAt(const LineSegment2d& segment, double parameter);
[[nodiscard]] GEOMETRY_API Vector2d TangentAt(const ArcSegment2d& segment, double parameter);
[[nodiscard]] GEOMETRY_API Vector2d TangentAt(const Segment2d& segment, double parameter);

[[nodiscard]] GEOMETRY_API Vector2d NormalAt(const LineSegment2d& segment, double parameter);
[[nodiscard]] GEOMETRY_API Vector2d NormalAt(const ArcSegment2d& segment, double parameter);
[[nodiscard]] GEOMETRY_API Vector2d NormalAt(const Segment2d& segment, double parameter);
} // namespace geometry::sdk
