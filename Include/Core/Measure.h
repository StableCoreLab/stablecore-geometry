#pragma once

#include <cstddef>

#include "Brep/SCBrepBody.h"
#include "Brep/SCBrepVertex.h"
#include "Brep/PolyhedronBody.h"
#include "Core/GeometryTypes.h"
#include "Export/GeometryExport.h"
#include "Geometry3d/ISCCurve3d.h"
#include "Geometry3d/SCCurveOnSurface.h"
#include "Geometry3d/ISCSurface.h"
#include "Geometry3d/TriangleMesh.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& lhs, const SCPoint3d& rhs);
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint3d& lhs, const SCPoint3d& rhs);

    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& point,
                                                      const SCLine3d& line,
                                                      const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint3d& point,
                                               const SCLine3d& line,
                                               const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& point, const SCPlane& plane);
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint3d& point, const SCPlane& plane);
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& point,
                                                      const ISCCurve3d& curve,
                                                      const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint3d& point,
                                               const ISCCurve3d& curve,
                                               const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& point,
                                                      const SCCurveOnSurface& curveOnSurface,
                                                      const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint3d& point,
                                               const SCCurveOnSurface& curveOnSurface,
                                               const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& point,
                                                      const ISCSurface& surface,
                                                      const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint3d& point,
                                               const ISCSurface& surface,
                                               const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& point,
                                                      const SCBrepFace& face,
                                                      const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint3d& point,
                                               const SCBrepFace& face,
                                               const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& point,
                                                      const SCBrepEdge& edge,
                                                      const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint3d& point,
                                               const SCBrepEdge& edge,
                                               const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& point,
                                                      const SCBrepVertex& vertex,
                                                      const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint3d& point,
                                               const SCBrepVertex& vertex,
                                               const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& point,
                                                      const SCBrepBody& body,
                                                      const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint3d& point,
                                               const SCBrepBody& body,
                                               const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& point,
                                                      const PolyhedronFace3d& face,
                                                      const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint3d& point,
                                               const PolyhedronFace3d& face,
                                               const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& point,
                                                      const PolyhedronBody& body,
                                                      const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint3d& point,
                                               const PolyhedronBody& body,
                                               const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& point,
                                                      const TriangleMesh& mesh,
                                                      const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint3d& point,
                                               const TriangleMesh& mesh,
                                               const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API double Length(const SCLineSegment3d& segment);
    [[nodiscard]] GEOMETRY_API double Length(const ISCCurve3d& curve, std::size_t sampleCount = 32);
    [[nodiscard]] GEOMETRY_API double Length(const SCCurveOnSurface& curveOnSurface);

    [[nodiscard]] GEOMETRY_API double Area(const SCTriangle3d& triangle);
    [[nodiscard]] GEOMETRY_API double Area(const PolyhedronFace3d& face, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double Area(const SCBrepFace& face, double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API double Area(const TriangleMesh& mesh);

    [[nodiscard]] GEOMETRY_API double Volume(const TriangleMesh& mesh);
    [[nodiscard]] GEOMETRY_API double Volume(const PolyhedronBody& body, double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API double Volume(const SCBrepBody& body, double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCBox3d Bounds(const ISCCurve3d& curve);
    [[nodiscard]] GEOMETRY_API SCBox3d Bounds(const ISCSurface& surface);
    [[nodiscard]] GEOMETRY_API SCBox3d Bounds(const SCCurveOnSurface& curveOnSurface);
    [[nodiscard]] GEOMETRY_API SCBox3d Bounds(const TriangleMesh& mesh);
    [[nodiscard]] GEOMETRY_API SCBox3d Bounds(const PolyhedronFace3d& face);
    [[nodiscard]] GEOMETRY_API SCBox3d Bounds(const PolyhedronBody& body);
    [[nodiscard]] GEOMETRY_API SCBox3d Bounds(const SCBrepVertex& vertex);
    [[nodiscard]] GEOMETRY_API SCBox3d Bounds(const SCBrepEdge& edge);
    [[nodiscard]] GEOMETRY_API SCBox3d Bounds(const SCBrepFace& face);
    [[nodiscard]] GEOMETRY_API SCBox3d Bounds(const SCBrepBody& body);
}  // namespace Geometry


