#pragma once

#include <cstddef>

#include "export/GeometryExport.h"
#include "sdk/BrepBody.h"
#include "sdk/CurveOnSurface.h"
#include "sdk/Curve3d.h"
#include "sdk/GeometryTypes.h"
#include "sdk/PolyhedronBody.h"
#include "sdk/Surface.h"
#include "sdk/TriangleMesh.h"

namespace geometry::sdk
{
[[nodiscard]] GEOMETRY_API double DistanceSquared(const Point3d& lhs, const Point3d& rhs);
[[nodiscard]] GEOMETRY_API double Distance(const Point3d& lhs, const Point3d& rhs);

[[nodiscard]] GEOMETRY_API double DistanceSquared(
    const Point3d& point,
    const Line3d& line,
    const GeometryTolerance3d& tolerance = {});
[[nodiscard]] GEOMETRY_API double Distance(
    const Point3d& point,
    const Line3d& line,
    const GeometryTolerance3d& tolerance = {});

[[nodiscard]] GEOMETRY_API double DistanceSquared(const Point3d& point, const Plane& plane);
[[nodiscard]] GEOMETRY_API double Distance(const Point3d& point, const Plane& plane);
[[nodiscard]] GEOMETRY_API double DistanceSquared(
    const Point3d& point,
    const Surface& surface,
    const GeometryTolerance3d& tolerance = {});
[[nodiscard]] GEOMETRY_API double Distance(
    const Point3d& point,
    const Surface& surface,
    const GeometryTolerance3d& tolerance = {});
[[nodiscard]] GEOMETRY_API double DistanceSquared(
    const Point3d& point,
    const BrepFace& face,
    const GeometryTolerance3d& tolerance = {});
[[nodiscard]] GEOMETRY_API double Distance(
    const Point3d& point,
    const BrepFace& face,
    const GeometryTolerance3d& tolerance = {});

[[nodiscard]] GEOMETRY_API double Length(const LineSegment3d& segment);
[[nodiscard]] GEOMETRY_API double Length(const Curve3d& curve, std::size_t sampleCount = 32);
[[nodiscard]] GEOMETRY_API double Length(const CurveOnSurface& curveOnSurface);

[[nodiscard]] GEOMETRY_API double Area(const Triangle3d& triangle);
[[nodiscard]] GEOMETRY_API double Area(const PolyhedronFace3d& face, const GeometryTolerance3d& tolerance = {});
[[nodiscard]] GEOMETRY_API double Area(const BrepFace& face, double eps = 1e-9);
[[nodiscard]] GEOMETRY_API double Area(const TriangleMesh& mesh);

[[nodiscard]] GEOMETRY_API double Volume(const TriangleMesh& mesh);
[[nodiscard]] GEOMETRY_API double Volume(const PolyhedronBody& body, double eps = 1e-9);
[[nodiscard]] GEOMETRY_API double Volume(const BrepBody& body, double eps = 1e-9);

[[nodiscard]] GEOMETRY_API Box3d Bounds(const Curve3d& curve);
[[nodiscard]] GEOMETRY_API Box3d Bounds(const Surface& surface);
[[nodiscard]] GEOMETRY_API Box3d Bounds(const CurveOnSurface& curveOnSurface);
[[nodiscard]] GEOMETRY_API Box3d Bounds(const TriangleMesh& mesh);
[[nodiscard]] GEOMETRY_API Box3d Bounds(const PolyhedronBody& body);
[[nodiscard]] GEOMETRY_API Box3d Bounds(const BrepBody& body);
} // namespace geometry::sdk
