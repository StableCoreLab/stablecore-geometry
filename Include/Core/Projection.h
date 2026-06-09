#pragma once

#include "Brep/SCBrepBody.h"
#include "Brep/SCBrepEdge.h"
#include "Brep/SCBrepFace.h"
#include "Brep/SCBrepVertex.h"
#include "Brep/PolyhedronBody.h"
#include "Brep/PolyhedronFace3d.h"
#include "Core/GeometryTypes.h"
#include "Export/GeometryExport.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Geometry2d/ISCSegment2d.h"
#include "Geometry3d/ISCSurface.h"
#include "Geometry3d/TriangleMesh.h"

namespace Geometry
{
    [[nodiscard]] GEOMETRY_API SCSegmentProjection2d ProjectPointToLineSegment(const SCPoint2d& point,
                                                                             const SCLineSegment2d& segment,
                                                                             bool clampToSegment = true);

    [[nodiscard]] GEOMETRY_API SCSegmentProjection2d ProjectPointToArcSegment(const SCPoint2d& point,
                                                                            const SCArcSegment2d& segment,
                                                                            bool clampToSegment = true);

    [[nodiscard]] GEOMETRY_API SCSegmentProjection2d ProjectPointToSegment(const SCPoint2d& point,
                                                                         const ISCSegment2d& segment,
                                                                         bool clampToSegment = true);

    [[nodiscard]] GEOMETRY_API SCSegmentProjection2d ProjectPointToSegment(const SCPoint2d& point,
                                                                         const SCPoint2d& segmentStart,
                                                                         const SCPoint2d& segmentEnd,
                                                                         bool clampToSegment = true);

    [[nodiscard]] GEOMETRY_API SCLineProjection3d ProjectPointToLine(const SCPoint3d& point,
                                                                   const SCLine3d& line,
                                                                   const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API SCPlaneProjection3d ProjectPointToPlane(const SCPoint3d& point,
                                                                     const SCPlane& plane,
                                                                     const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API SCCurveProjection3d ProjectPointToCurve(const SCPoint3d& point,
                                                                     const ISCCurve3d& curve,
                                                                     const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API SCCurveOnSurfaceProjection3d ProjectPointToCurveOnSurface(
        const SCPoint3d& point, const SCCurveOnSurface& curveOnSurface, const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API SCSurfaceProjection3d ProjectPointToSurface(const SCPoint3d& point,
                                                                         const ISCSurface& surface,
                                                                         const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API SCBrepFaceProjection3d ProjectPointToBrepFace(const SCPoint3d& point,
                                                                           const SCBrepFace& face,
                                                                           const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API SCBrepEdgeProjection3d ProjectPointToBrepEdge(const SCPoint3d& point,
                                                                           const SCBrepEdge& edge,
                                                                           const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API SCBrepVertexProjection3d
    ProjectPointToBrepVertex(const SCPoint3d& point, const SCBrepVertex& vertex, const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API SCBrepBodyProjection3d ProjectPointToBrepBody(const SCPoint3d& point,
                                                                           const SCBrepBody& body,
                                                                           const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API SCPolyhedronFaceProjection3d ProjectPointToPolyhedronFace(
        const SCPoint3d& point, const PolyhedronFace3d& face, const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API SCPolyhedronBodyProjection3d ProjectPointToPolyhedronBody(
        const SCPoint3d& point, const PolyhedronBody& body, const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API SCTriangleMeshProjection3d ProjectPointToTriangleMesh(
        const SCPoint3d& point, const TriangleMesh& mesh, const SCGeometryTolerance3d& tolerance = {});

    struct GEOMETRY_API SCFaceProjection3d
    {
        bool success{false};
        SCPolygon2d polygon{};
        SCPoint3d origin{};
        SCVector3d uAxis{};
        SCVector3d vAxis{};
    };

    [[nodiscard]] GEOMETRY_API SCFaceProjection3d ProjectFaceToPolygon2d(const PolyhedronFace3d& face,
                                                                       const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API double ParameterAtLength(const SCLineSegment2d& segment,
                                                        double length,
                                                        bool clampToSegment = false);
    [[nodiscard]] GEOMETRY_API double ParameterAtLength(const SCArcSegment2d& segment,
                                                        double length,
                                                        bool clampToSegment = false);
    [[nodiscard]] GEOMETRY_API double ParameterAtLength(const ISCSegment2d& segment,
                                                        double length,
                                                        bool clampToSegment = false);

    [[nodiscard]] GEOMETRY_API SCVector2d TangentAt(const SCLineSegment2d& segment, double parameter);
    [[nodiscard]] GEOMETRY_API SCVector2d TangentAt(const SCArcSegment2d& segment, double parameter);
    [[nodiscard]] GEOMETRY_API SCVector2d TangentAt(const ISCSegment2d& segment, double parameter);

    [[nodiscard]] GEOMETRY_API SCVector2d NormalAt(const SCLineSegment2d& segment, double parameter);
    [[nodiscard]] GEOMETRY_API SCVector2d NormalAt(const SCArcSegment2d& segment, double parameter);
    [[nodiscard]] GEOMETRY_API SCVector2d NormalAt(const ISCSegment2d& segment, double parameter);
}  // namespace Geometry


