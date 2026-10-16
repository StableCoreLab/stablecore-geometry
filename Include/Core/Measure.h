#pragma once

#include <cstddef>

#include "Core/GeometryEvidence.h"
#include "Core/GeometryTypes.h"
#include "Export/GeometryExport.h"
#include "Geometry3d/SCCurveOnSurface.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    class ISCCurve3d;
    class ISCSurface;
    class SCBrepBody;
    class SCBrepEdge;
    class SCBrepFace;
    class SCBrepVertex;
    class PolyhedronBody;
    class PolyhedronFace3d;
    class TriangleMesh;

    struct GEOMETRY_API SCDistancePayload3d
    {
        double distance{0.0};
        double distanceSquared{0.0};
        bool distanceValid{false};
        bool distanceSquaredValid{false};
    };

    struct GEOMETRY_API SCDistanceResult3dWithStatus
    {
        SCGeometryOperationState state{SCGeometryOperationState::InvalidInput};
        bool payloadValid{false};
        SCDistancePayload3d payload{};
        SCGeometryEvidenceStatus evidence{};
    };

    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& lhs, const SCPoint3d& rhs);
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint3d& lhs, const SCPoint3d& rhs);
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceSquaredWithStatus(
        const SCPoint3d& lhs, const SCPoint3d& rhs);
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceWithStatus(
        const SCPoint3d& lhs, const SCPoint3d& rhs);

    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& point,
                                                      const SCLine3d& line,
                                                      const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint3d& point,
                                               const SCLine3d& line,
                                               const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceSquaredWithStatus(
        const SCPoint3d& point, const SCLine3d& line, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceWithStatus(
        const SCPoint3d& point, const SCLine3d& line, const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API double DistanceSquared(const SCPoint3d& point, const SCPlane& plane);
    [[nodiscard]] GEOMETRY_API double Distance(const SCPoint3d& point, const SCPlane& plane);
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceSquaredWithStatus(
        const SCPoint3d& point, const SCPlane& plane, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceWithStatus(
        const SCPoint3d& point, const SCPlane& plane, const SCGeometryTolerance3d& tolerance = {});
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
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceSquaredWithStatus(
        const SCPoint3d& point, const SCBrepVertex& vertex, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceWithStatus(
        const SCPoint3d& point, const SCBrepVertex& vertex, const SCGeometryTolerance3d& tolerance = {});
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

    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceWithStatus(
        const SCPoint3d& point, const ISCCurve3d& target, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceSquaredWithStatus(
        const SCPoint3d& point, const ISCCurve3d& target, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceWithStatus(
        const SCPoint3d& point, const SCCurveOnSurface& target, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceSquaredWithStatus(
        const SCPoint3d& point, const SCCurveOnSurface& target, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceWithStatus(
        const SCPoint3d& point, const ISCSurface& target, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceSquaredWithStatus(
        const SCPoint3d& point, const ISCSurface& target, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceWithStatus(
        const SCPoint3d& point, const SCBrepFace& target, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceSquaredWithStatus(
        const SCPoint3d& point, const SCBrepFace& target, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceWithStatus(
        const SCPoint3d& point, const SCBrepEdge& target, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceSquaredWithStatus(
        const SCPoint3d& point, const SCBrepEdge& target, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceWithStatus(
        const SCPoint3d& point, const SCBrepBody& target, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceSquaredWithStatus(
        const SCPoint3d& point, const SCBrepBody& target, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceWithStatus(
        const SCPoint3d& point, const PolyhedronFace3d& target, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceSquaredWithStatus(
        const SCPoint3d& point, const PolyhedronFace3d& target, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceWithStatus(
        const SCPoint3d& point, const PolyhedronBody& target, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceSquaredWithStatus(
        const SCPoint3d& point, const PolyhedronBody& target, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceWithStatus(
        const SCPoint3d& point, const TriangleMesh& target, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API SCDistanceResult3dWithStatus DistanceSquaredWithStatus(
        const SCPoint3d& point, const TriangleMesh& target, const SCGeometryTolerance3d& tolerance = {});

    [[nodiscard]] GEOMETRY_API double Length(const SCLineSegment3d& segment);
    [[nodiscard]] GEOMETRY_API double Length(const ISCCurve3d& curve, std::size_t sampleCount = 32);
    // 对实际 UV 边映射作 32 段保底及最多 8 层弦差细分的近似长度，包含闭合边。
    // 有限采样不提供任意曲面的全局误差证明，不用于拓扑排除或硬剪枝。
    // 实际位置或累积长度不可表示时返回 +infinity。
    [[nodiscard]] GEOMETRY_API double Length(const SCCurveOnSurface& curveOnSurface);

    [[nodiscard]] GEOMETRY_API double Area(const SCTriangle3d& triangle);
    [[nodiscard]] GEOMETRY_API double Area(const PolyhedronFace3d& face, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double Area(const SCBrepFace& face, const SCGeometryTolerance3d& tolerance = {});
    [[deprecated("Use the overload taking SCGeometryTolerance3d")]]
    [[nodiscard]] GEOMETRY_API double Area(const SCBrepFace& face, double eps);
    [[nodiscard]] GEOMETRY_API double Area(const TriangleMesh& mesh);

    [[nodiscard]] GEOMETRY_API double Volume(const TriangleMesh& mesh);
    [[nodiscard]] GEOMETRY_API double Volume(const PolyhedronBody& body, const SCGeometryTolerance3d& tolerance = {});
    [[nodiscard]] GEOMETRY_API double Volume(const SCBrepBody& body, const SCGeometryTolerance3d& tolerance = {});
    [[deprecated("Use the overload taking SCGeometryTolerance3d")]]
    [[nodiscard]] GEOMETRY_API double Volume(const PolyhedronBody& body, double eps);
    [[deprecated("Use the overload taking SCGeometryTolerance3d")]]
    [[nodiscard]] GEOMETRY_API double Volume(const SCBrepBody& body, double eps);

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


