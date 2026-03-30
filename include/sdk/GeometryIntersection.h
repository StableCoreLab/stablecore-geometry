#pragma once

#include "export/GeometryExport.h"
#include "sdk/ArcSegment2d.h"
#include "sdk/GeometryResults.h"
#include "sdk/LineSegment2d.h"
#include "sdk/Segment2d.h"
#include "sdk/BrepBody.h"
#include "sdk/PolyhedronBody.h"
#include "sdk/Surface.h"

namespace geometry::sdk
{
[[nodiscard]] GEOMETRY_API SegmentIntersection2d Intersect(
    const LineSegment2d& first,
    const LineSegment2d& second,
    double eps = 1e-9);
[[nodiscard]] GEOMETRY_API SegmentIntersection2d Intersect(
    const LineSegment2d& first,
    const ArcSegment2d& second,
    double eps = 1e-9);
[[nodiscard]] GEOMETRY_API SegmentIntersection2d Intersect(
    const ArcSegment2d& first,
    const ArcSegment2d& second,
    double eps = 1e-9);
[[nodiscard]] GEOMETRY_API SegmentIntersection2d Intersect(
    const Segment2d& first,
    const Segment2d& second,
    double eps = 1e-9);

[[nodiscard]] GEOMETRY_API LinePlaneIntersection3d Intersect(
    const Line3d& line,
    const Plane& plane,
    const GeometryTolerance3d& tolerance = {});
[[nodiscard]] GEOMETRY_API LineCurveIntersection3d Intersect(
    const Line3d& line,
    const Curve3d& curve,
    const GeometryTolerance3d& tolerance = {});
[[nodiscard]] GEOMETRY_API LineCurveOnSurfaceIntersection3d Intersect(
    const Line3d& line,
    const CurveOnSurface& curveOnSurface,
    const GeometryTolerance3d& tolerance = {});
[[nodiscard]] GEOMETRY_API LineSurfaceIntersection3d Intersect(
    const Line3d& line,
    const Surface& surface,
    const GeometryTolerance3d& tolerance = {});
[[nodiscard]] GEOMETRY_API LineBrepEdgeIntersection3d Intersect(
    const Line3d& line,
    const BrepEdge& edge,
    const GeometryTolerance3d& tolerance = {});
[[nodiscard]] GEOMETRY_API LineBrepFaceIntersection3d Intersect(
    const Line3d& line,
    const BrepFace& face,
    const GeometryTolerance3d& tolerance = {});
[[nodiscard]] GEOMETRY_API LineBrepBodyIntersection3d Intersect(
    const Line3d& line,
    const BrepBody& body,
    const GeometryTolerance3d& tolerance = {});
[[nodiscard]] GEOMETRY_API LinePolyhedronFaceIntersection3d Intersect(
    const Line3d& line,
    const PolyhedronFace3d& face,
    const GeometryTolerance3d& tolerance = {});
[[nodiscard]] GEOMETRY_API LinePolyhedronBodyIntersection3d Intersect(
    const Line3d& line,
    const PolyhedronBody& body,
    const GeometryTolerance3d& tolerance = {});
[[nodiscard]] GEOMETRY_API PlanePlaneIntersection3d Intersect(
    const Plane& first,
    const Plane& second,
    const GeometryTolerance3d& tolerance = {});

[[nodiscard]] GEOMETRY_API bool HasIntersection(
    const Segment2d& first,
    const Segment2d& second,
    double eps = 1e-9);

[[nodiscard]] GEOMETRY_API ClosestPoints2d ClosestPoints(
    const LineSegment2d& first,
    const LineSegment2d& second);
[[nodiscard]] GEOMETRY_API ClosestPoints2d ClosestPoints(
    const LineSegment2d& first,
    const ArcSegment2d& second);
[[nodiscard]] GEOMETRY_API ClosestPoints2d ClosestPoints(
    const ArcSegment2d& first,
    const ArcSegment2d& second);
[[nodiscard]] GEOMETRY_API ClosestPoints2d ClosestPoints(
    const Segment2d& first,
    const Segment2d& second);
} // namespace geometry::sdk
