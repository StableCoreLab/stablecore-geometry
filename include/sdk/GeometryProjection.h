#pragma once

#include "export/GeometryExport.h"
#include "sdk/ArcSegment2d.h"
#include "sdk/GeometryTypes.h"
#include "sdk/BrepBody.h"
#include "sdk/LineSegment2d.h"
#include "sdk/Polygon2d.h"
#include "sdk/BrepVertex.h"
#include "sdk/BrepEdge.h"
#include "sdk/PolyhedronBody.h"
#include "sdk/PolyhedronFace3d.h"
#include "sdk/BrepFace.h"
#include "sdk/Segment2d.h"
#include "sdk/Surface.h"
#include "sdk/TriangleMesh.h"

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

[[nodiscard]] GEOMETRY_API CurveProjection3d ProjectPointToCurve(
    const Point3d& point,
    const Curve3d& curve,
    const GeometryTolerance3d& tolerance = {});

[[nodiscard]] GEOMETRY_API CurveOnSurfaceProjection3d ProjectPointToCurveOnSurface(
    const Point3d& point,
    const CurveOnSurface& curveOnSurface,
    const GeometryTolerance3d& tolerance = {});

[[nodiscard]] GEOMETRY_API SurfaceProjection3d ProjectPointToSurface(
    const Point3d& point,
    const Surface& surface,
    const GeometryTolerance3d& tolerance = {});

[[nodiscard]] GEOMETRY_API BrepFaceProjection3d ProjectPointToBrepFace(
    const Point3d& point,
    const BrepFace& face,
    const GeometryTolerance3d& tolerance = {});

[[nodiscard]] GEOMETRY_API BrepEdgeProjection3d ProjectPointToBrepEdge(
    const Point3d& point,
    const BrepEdge& edge,
    const GeometryTolerance3d& tolerance = {});

[[nodiscard]] GEOMETRY_API BrepVertexProjection3d ProjectPointToBrepVertex(
    const Point3d& point,
    const BrepVertex& vertex,
    const GeometryTolerance3d& tolerance = {});

[[nodiscard]] GEOMETRY_API BrepBodyProjection3d ProjectPointToBrepBody(
    const Point3d& point,
    const BrepBody& body,
    const GeometryTolerance3d& tolerance = {});

[[nodiscard]] GEOMETRY_API PolyhedronFaceProjection3d ProjectPointToPolyhedronFace(
    const Point3d& point,
    const PolyhedronFace3d& face,
    const GeometryTolerance3d& tolerance = {});

[[nodiscard]] GEOMETRY_API PolyhedronBodyProjection3d ProjectPointToPolyhedronBody(
    const Point3d& point,
    const PolyhedronBody& body,
    const GeometryTolerance3d& tolerance = {});

[[nodiscard]] GEOMETRY_API TriangleMeshProjection3d ProjectPointToTriangleMesh(
    const Point3d& point,
    const TriangleMesh& mesh,
    const GeometryTolerance3d& tolerance = {});

struct GEOMETRY_API FaceProjection3d
{
    bool success{false};
    Polygon2d polygon{};
    Point3d origin{};
    Vector3d uAxis{};
    Vector3d vAxis{};
};

[[nodiscard]] GEOMETRY_API FaceProjection3d ProjectFaceToPolygon2d(
    const PolyhedronFace3d& face,
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
