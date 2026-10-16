#pragma once

#include "Core/GeometryEvidence.h"
#include "Core/GeometryTypes.h"
#include "Export/GeometryExport.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCLine2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/SCPolygon2d.h"
#include "Geometry2d/ISCSegment2d.h"

namespace Geometry
{
    class ISCCurve3d;
    class ISCSurface;
    class SCCurveOnSurface;
    class SCBrepBody;
    class SCBrepEdge;
    class SCBrepFace;
    class SCBrepVertex;
    class PolyhedronBody;
    class PolyhedronFace3d;
    class TriangleMesh;

    struct GEOMETRY_API SCBrepVertexProjection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCBrepVertexProjection3d payload{};
        SCGeometryEvidenceStatus evidence{};
    };

    struct GEOMETRY_API SCBrepFaceProjection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCBrepFaceProjection3d payload{};
        SCGeometryEvidenceStatus evidence{};
        SCGeometrySearchStatistics statistics{};
    };

    struct GEOMETRY_API SCPolyhedronFaceProjection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCPolyhedronFaceProjection3d payload{};
        SCGeometryEvidenceStatus evidence{};
    };

    struct SCBrepFaceProjectionRecord3d
    {
        std::size_t shellIndex{0};
        std::size_t localFaceIndex{0};
        std::size_t faceIndex{0};
        SCBrepFaceProjection3dWithStatus result{};
    };
    struct GEOMETRY_API SCBrepBodyProjection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCBrepBodyProjection3d payload{};
        SCGeometryEvidenceStatus evidence{};
        std::vector<SCBrepFaceProjectionRecord3d> faces{};
    };
    struct GEOMETRY_API SCPolyhedronBodyProjection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCPolyhedronBodyProjection3d payload{};
        SCGeometryEvidenceStatus evidence{};
        std::vector<SCPolyhedronFaceProjection3dWithStatus> faces{};
    };
    [[nodiscard]] GEOMETRY_API SCBrepBodyProjection3dWithStatus ProjectPointToBrepBodyWithStatus(
        const SCPoint3d& point, const SCBrepBody& target, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCPolyhedronBodyProjection3dWithStatus ProjectPointToPolyhedronBodyWithStatus(
        const SCPoint3d& point, const PolyhedronBody& target, const SCGeometryTolerance3d& tolerance = {});

    // success 只表示可读取候选；最优性必须单独检查 evidence.optimality。
    struct GEOMETRY_API SCCurveProjection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCCurveProjection3d payload{};
        SCGeometryEvidenceStatus evidence{};
    };
    [[nodiscard]] GEOMETRY_API SCCurveProjection3dWithStatus ProjectPointToCurveWithStatus(
        const SCPoint3d& point, const ISCCurve3d& target, const SCGeometryTolerance3d& tolerance = {});

    struct GEOMETRY_API SCCurveOnSurfaceProjection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCCurveOnSurfaceProjection3d payload{};
        SCGeometryEvidenceStatus evidence{};
    };
    [[nodiscard]] GEOMETRY_API SCCurveOnSurfaceProjection3dWithStatus ProjectPointToCurveOnSurfaceWithStatus(
        const SCPoint3d& point, const SCCurveOnSurface& target, const SCGeometryTolerance3d& tolerance = {});

    struct GEOMETRY_API SCSurfaceProjection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCSurfaceProjection3d payload{};
        SCGeometryEvidenceStatus evidence{};
        SCGeometrySearchStatistics statistics{};
    };
    [[nodiscard]] GEOMETRY_API SCSurfaceProjection3dWithStatus ProjectPointToSurfaceWithStatus(
        const SCPoint3d& point, const ISCSurface& target, const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API SCBrepFaceProjection3dWithStatus ProjectPointToBrepFaceWithStatus(
        const SCPoint3d& point, const SCBrepFace& target, const SCGeometryTolerance3d& tolerance = {});

    struct GEOMETRY_API SCBrepEdgeProjection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCBrepEdgeProjection3d payload{};
        SCGeometryEvidenceStatus evidence{};
    };
    [[nodiscard]] GEOMETRY_API SCBrepEdgeProjection3dWithStatus ProjectPointToBrepEdgeWithStatus(
        const SCPoint3d& point, const SCBrepEdge& target, const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API SCPolyhedronFaceProjection3dWithStatus ProjectPointToPolyhedronFaceWithStatus(
        const SCPoint3d& point, const PolyhedronFace3d& target, const SCGeometryTolerance3d& tolerance = {});

    struct GEOMETRY_API SCTriangleMeshProjection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCTriangleMeshProjection3d payload{};
        SCGeometryEvidenceStatus evidence{};
    };
    [[nodiscard]] GEOMETRY_API SCTriangleMeshProjection3dWithStatus ProjectPointToTriangleMeshWithStatus(
        const SCPoint3d& point, const TriangleMesh& target, const SCGeometryTolerance3d& tolerance = {});

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
    [[nodiscard]] GEOMETRY_API SCSegmentProjection3d ProjectPointToLineSegment(const SCPoint3d& point,
                                                                             const SCLineSegment3d& segment,
                                                                             bool clampToSegment = true,
                                                                             const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCSegmentProjection3d ProjectPointToLineSegment(const SCPoint3d& point,
                                                                             const SCPoint3d& segmentStart,
                                                                             const SCPoint3d& segmentEnd,
                                                                             bool clampToSegment = true,
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
    [[nodiscard]] GEOMETRY_API SCBrepVertexProjection3dWithStatus
    ProjectPointToBrepVertexWithStatus(const SCPoint3d& point, const SCBrepVertex& vertex,
                                      const SCGeometryTolerance3d& tolerance = {});

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

