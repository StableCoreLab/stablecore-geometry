#pragma once

#include "Core/GeometryEvidence.h"
#include "Core/GeometryTypes.h"

#include "Geometry2d/SCLine2d.h"
#include "Core/Results.h"
#include "Export/GeometryExport.h"
#include "Geometry2d/SCArcSegment2d.h"
#include "Geometry2d/SCPolyline2d.h"
#include "Geometry2d/SCLineSegment2d.h"
#include "Geometry2d/ISCSegment2d.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    class SCBrepBody;
    class SCBrepEdge;
    class SCBrepFace;
    class SCBrepVertex;
    class PolyhedronBody;
    class PolyhedronFace3d;
    class TriangleMesh;
    class ISCCurve3d;
    class ISCSurface;
    class SCCurveOnSurface;

    struct GEOMETRY_API SCLinePlaneIntersection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCLinePlaneIntersection3d payload{};
        SCGeometryEvidenceStatus evidence{};
    };
    struct GEOMETRY_API SCPlanePlaneIntersection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCPlanePlaneIntersection3d payload{};
        SCGeometryEvidenceStatus evidence{};
    };
    struct GEOMETRY_API SCLineBrepEdgeIntersection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCLineBrepEdgeIntersection3d payload{};
        SCGeometryEvidenceStatus evidence{};
    };
    struct GEOMETRY_API SCLineBrepVertexIntersection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCLineBrepVertexIntersection3d payload{};
        SCGeometryEvidenceStatus evidence{};
    };
    struct GEOMETRY_API SCPlaneBrepEdgeIntersection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCPlaneBrepEdgeIntersection3d payload{};
        SCGeometryEvidenceStatus evidence{};
    };
    struct GEOMETRY_API SCPlaneBrepVertexIntersection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCPlaneBrepVertexIntersection3d payload{};
        SCGeometryEvidenceStatus evidence{};
    };
    [[nodiscard]] GEOMETRY_API SCLinePlaneIntersection3dWithStatus IntersectLinePlaneWithStatus(
        const SCLine3d& line, const SCPlane& plane, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCPlanePlaneIntersection3dWithStatus IntersectPlanePlaneWithStatus(
        const SCPlane& first, const SCPlane& second, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCLineBrepEdgeIntersection3dWithStatus IntersectLineBrepEdgeWithStatus(
        const SCLine3d& line, const SCBrepEdge& edge, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCLineBrepVertexIntersection3dWithStatus IntersectLineBrepVertexWithStatus(
        const SCLine3d& line, const SCBrepVertex& vertex, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCPlaneBrepEdgeIntersection3dWithStatus IntersectPlaneBrepEdgeWithStatus(
        const SCPlane& plane, const SCBrepEdge& edge, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCPlaneBrepVertexIntersection3dWithStatus IntersectPlaneBrepVertexWithStatus(
        const SCPlane& plane, const SCBrepVertex& vertex, const SCGeometryTolerance3d& tolerance = {});

    struct GEOMETRY_API SCLineSurfaceIntersection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCLineSurfaceIntersection3d payload{};
        SCGeometryEvidenceStatus evidence{};
    };
    [[nodiscard]] GEOMETRY_API SCLineSurfaceIntersection3dWithStatus IntersectLineSurfaceWithStatus(
        const SCLine3d& primitive, const ISCSurface& target, const SCGeometryTolerance3d& tolerance = {});

    struct GEOMETRY_API SCLineCurveIntersection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCLineCurveIntersection3d payload{};
        SCGeometryEvidenceStatus evidence{};
    };
    [[nodiscard]] GEOMETRY_API SCLineCurveIntersection3dWithStatus IntersectLineCurveWithStatus(
        const SCLine3d& primitive, const ISCCurve3d& target, const SCGeometryTolerance3d& tolerance = {});

    struct GEOMETRY_API SCLineCurveOnSurfaceIntersection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCLineCurveOnSurfaceIntersection3d payload{};
        SCGeometryEvidenceStatus evidence{};
    };
    [[nodiscard]] GEOMETRY_API SCLineCurveOnSurfaceIntersection3dWithStatus IntersectLineCurveOnSurfaceWithStatus(
        const SCLine3d& primitive, const SCCurveOnSurface& target, const SCGeometryTolerance3d& tolerance = {});

    struct GEOMETRY_API SCPlaneCurveIntersection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCPlaneCurveIntersection3d payload{};
        SCGeometryEvidenceStatus evidence{};
    };
    [[nodiscard]] GEOMETRY_API SCPlaneCurveIntersection3dWithStatus IntersectPlaneCurveWithStatus(
        const SCPlane& primitive, const ISCCurve3d& target, const SCGeometryTolerance3d& tolerance = {});

    struct GEOMETRY_API SCPlaneCurveOnSurfaceIntersection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCPlaneCurveOnSurfaceIntersection3d payload{};
        SCGeometryEvidenceStatus evidence{};
    };
    [[nodiscard]] GEOMETRY_API SCPlaneCurveOnSurfaceIntersection3dWithStatus IntersectPlaneCurveOnSurfaceWithStatus(
        const SCPlane& primitive, const SCCurveOnSurface& target, const SCGeometryTolerance3d& tolerance = {});

    struct GEOMETRY_API SCLinePolyhedronFaceIntersection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCLinePolyhedronFaceIntersection3d payload{};
        SCGeometryEvidenceStatus evidence{};
        std::vector<SCLinePolyhedronFaceIntersection3d> hits{};
        std::vector<SCGeometryEvidenceStatus> hitEvidence{};
    };
    [[nodiscard]] GEOMETRY_API SCLinePolyhedronFaceIntersection3dWithStatus IntersectLinePolyhedronFaceWithStatus(
        const SCLine3d& primitive, const PolyhedronFace3d& target, const SCGeometryTolerance3d& tolerance = {});

    struct GEOMETRY_API SCLineBrepFaceIntersection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCLineBrepFaceIntersection3d payload{};
        SCGeometryEvidenceStatus evidence{};
        std::vector<SCLineBrepFaceIntersection3d> hits{};
        std::vector<SCGeometryEvidenceStatus> hitEvidence{};
    };
    struct SCLineBrepFaceRecord3d
    {
        std::size_t shellIndex{0}, localFaceIndex{0}, faceIndex{0};
        SCLineBrepFaceIntersection3dWithStatus result{};
    };
    struct GEOMETRY_API SCLineBrepBodyIntersection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCLineBrepBodyIntersection3d payload{};
        SCGeometryEvidenceStatus evidence{};
        std::vector<SCLineBrepFaceRecord3d> faces{};
        std::vector<SCGeometryEvidenceStatus> hitEvidence{};
    };
    struct GEOMETRY_API SCLinePolyhedronBodyIntersection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCLinePolyhedronBodyIntersection3d payload{};
        SCGeometryEvidenceStatus evidence{};
        std::vector<SCLinePolyhedronFaceIntersection3dWithStatus> faces{};
        std::vector<SCGeometryEvidenceStatus> hitEvidence{};
    };
    struct GEOMETRY_API SCLineTriangleMeshIntersection3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCLineTriangleMeshIntersection3d payload{};
        SCGeometryEvidenceStatus evidence{};
        std::vector<SCGeometryEvidenceStatus> triangles{};
        std::vector<SCGeometryEvidenceStatus> hitEvidence{};
    };
    [[nodiscard]] GEOMETRY_API SCLineBrepFaceIntersection3dWithStatus IntersectLineBrepFaceWithStatus(
        const SCLine3d& line, const SCBrepFace& face, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCLineBrepBodyIntersection3dWithStatus IntersectLineBrepBodyWithStatus(
        const SCLine3d& line, const SCBrepBody& body, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCLinePolyhedronBodyIntersection3dWithStatus IntersectLinePolyhedronBodyWithStatus(
        const SCLine3d& line, const PolyhedronBody& body, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCLineTriangleMeshIntersection3dWithStatus IntersectLineTriangleMeshWithStatus(
        const SCLine3d& line, const TriangleMesh& mesh, const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API SCSegmentIntersection2d Intersect(const SCLineSegment2d& first,
                                                               const SCLineSegment2d& second,
                                                               double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCLineIntersection2d Intersect(const SCLine2d& first,
                                                              const SCLine2d& second,
                                                              double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCLineIntersection2d Intersect(const SCLine2d& first,
                                                              const SCLineSegment2d& second,
                                                              double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCLineIntersection2d Intersect(const SCLineSegment2d& first,
                                                              const SCLine2d& second,
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
    [[nodiscard]] GEOMETRY_API std::vector<SCPolylineIntersectionPoint2d> Intersect(const SCPolyline2d& first,
                                                                                   const SCPolyline2d& second,
                                                                                   double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API std::vector<SCPolylineIntersectionPoint2d> Intersect(const SCPolyline2d& polyline,
                                                                                   const ISCSegment2d& segment,
                                                                                   double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API std::vector<SCPolylineIntersectionPoint2d> Intersect(const ISCSegment2d& segment,
                                                                                   const SCPolyline2d& polyline,
                                                                                   double eps = Geometry::kDefaultEpsilon);
    [[nodiscard]] GEOMETRY_API SCExtendedIntersection2d IntersectExtended(const ISCSegment2d& first,
                                                                         const ISCSegment2d& second,
                                                                         SCExtensionPolicy policy = SCExtensionPolicy::ExtendBoth,
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

