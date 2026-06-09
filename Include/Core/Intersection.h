#pragma once

#include "Brep/SCBrepBody.h"
#include "Brep/SCBrepVertex.h"
#include "Brep/PolyhedronBody.h"
#include "Core/Results.h"
#include "Export/GeometryExport.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/ISCSegment2d.h"
#include "Geometry3d/ISCSurface.h"
#include "Geometry3d/TriangleMesh.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    [[nodiscard]] GEOMETRY_API SCSegmentIntersection2d Intersect(const SCLineSegment2d& first,
                                                               const SCLineSegment2d& second,
                                                               double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCSegmentIntersection2d Intersect(const SCLineSegment2d& first,
                                                               const SCArcSegment2d& second,
                                                               double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCSegmentIntersection2d Intersect(const SCArcSegment2d& first,
                                                               const SCArcSegment2d& second,
                                                               double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCSegmentIntersection2d Intersect(const ISCSegment2d& first,
                                                               const ISCSegment2d& second,
                                                               double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCLinePlaneIntersection3d Intersect(const SCLine3d& line,
                                                                 const SCPlane& plane,
                                                                 const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCLineCurveIntersection3d Intersect(const SCLine3d& line,
                                                                 const ISCCurve3d& curve,
                                                                 const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCLineCurveOnSurfaceIntersection3d Intersect(const SCLine3d& line,
                                                                          const SCCurveOnSurface& curveOnSurface,
                                                                          const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCLineSurfaceIntersection3d Intersect(const SCLine3d& line,
                                                                   const ISCSurface& surface,
                                                                   const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCLineBrepEdgeIntersection3d Intersect(const SCLine3d& line,
                                                                    const SCBrepEdge& edge,
                                                                    const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCLineBrepVertexIntersection3d Intersect(const SCLine3d& line,
                                                                      const SCBrepVertex& vertex,
                                                                      const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCLineBrepFaceIntersection3d Intersect(const SCLine3d& line,
                                                                    const SCBrepFace& face,
                                                                    const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCLineBrepBodyIntersection3d Intersect(const SCLine3d& line,
                                                                    const SCBrepBody& body,
                                                                    const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCLinePolyhedronFaceIntersection3d Intersect(const SCLine3d& line,
                                                                          const PolyhedronFace3d& face,
                                                                          const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCLinePolyhedronBodyIntersection3d Intersect(const SCLine3d& line,
                                                                          const PolyhedronBody& body,
                                                                          const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCLineTriangleMeshIntersection3d Intersect(const SCLine3d& line,
                                                                        const TriangleMesh& mesh,
                                                                        const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCPlaneCurveIntersection3d Intersect(const SCPlane& plane,
                                                                  const ISCCurve3d& curve,
                                                                  const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCPlaneCurveOnSurfaceIntersection3d Intersect(const SCPlane& plane,
                                                                           const SCCurveOnSurface& curveOnSurface,
                                                                           const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCPlaneBrepEdgeIntersection3d Intersect(const SCPlane& plane,
                                                                     const SCBrepEdge& edge,
                                                                     const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCPlaneBrepVertexIntersection3d Intersect(const SCPlane& plane,
                                                                       const SCBrepVertex& vertex,
                                                                       const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCPlanePlaneIntersection3d Intersect(const SCPlane& first,
                                                                  const SCPlane& second,
                                                                  const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API bool HasIntersection(const ISCSegment2d& first,
                                                    const ISCSegment2d& second,
                                                    double eps = Geometry::kDefaultEpsilon);

    [[nodiscard]] GEOMETRY_API SCClosestPoints2d ClosestPoints(const SCLineSegment2d& first, const SCLineSegment2d& second);
    [[nodiscard]] GEOMETRY_API SCClosestPoints2d ClosestPoints(const SCLineSegment2d& first, const SCArcSegment2d& second);
    [[nodiscard]] GEOMETRY_API SCClosestPoints2d ClosestPoints(const SCArcSegment2d& first, const SCArcSegment2d& second);
    [[nodiscard]] GEOMETRY_API SCClosestPoints2d ClosestPoints(const ISCSegment2d& first, const ISCSegment2d& second);
}  // namespace Geometry


