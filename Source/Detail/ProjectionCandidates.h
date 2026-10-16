#pragma once

#include "Core/Projection.h"

namespace Geometry::Detail
{
    // 仅供求解器和距离入口使用，success 表示有限一阶候选；平方可为 infinity。
    [[nodiscard]] SCCurveProjection3d SolveCurveProjection(const SCPoint3d&, const ISCCurve3d&, const SCGeometryTolerance3d&);
    [[nodiscard]] SCCurveOnSurfaceProjection3d SolveCurveOnSurfaceProjection(const SCPoint3d&, const SCCurveOnSurface&, const SCGeometryTolerance3d&,
                                                                             bool* analyticComplete = nullptr);
    [[nodiscard]] SCBrepFaceProjection3d SolveFaceProjection(const SCPoint3d&, const SCBrepFace&, const SCGeometryTolerance3d&,
                                                           bool bounded = false, SCGeometryEvidenceStatus* evidence = nullptr,
                                                           SCGeometrySearchStatistics* statistics = nullptr);
    [[nodiscard]] SCBrepEdgeProjection3d SolveEdgeProjection(const SCPoint3d&, const SCBrepEdge&, const SCGeometryTolerance3d&);
    [[nodiscard]] SCPolyhedronFaceProjection3d SolvePolyFaceProjection(const SCPoint3d&, const PolyhedronFace3d&, const SCGeometryTolerance3d&);
    [[nodiscard]] SCTriangleMeshProjection3d SolveMeshProjection(const SCPoint3d&, const TriangleMesh&, const SCGeometryTolerance3d&);

    struct BrepFaceCandidateRecord
    {
        std::size_t shellIndex{0}, localFaceIndex{0}, faceIndex{0};
        SCBrepFaceProjection3d candidate{};
        SCGeometryEvidenceStatus evidence{};
        SCGeometrySearchStatistics statistics{};
    };
    struct BrepBodyCandidate
    {
        bool validInput{false}, allFacesAvailable{true};
        SCBrepBodyProjection3d candidate{};
        SCGeometryEvidenceStatus evidence{};
        std::vector<BrepFaceCandidateRecord> faces{};
    };
    struct PolyhedronBodyCandidate
    {
        bool validInput{false}, allFacesAvailable{true};
        SCPolyhedronBodyProjection3d candidate{};
        SCGeometryEvidenceStatus evidence{};
        std::vector<SCPolyhedronFaceProjection3d> faces{};
    };
    [[nodiscard]] BrepBodyCandidate SolveBodyProjection(const SCPoint3d&, const SCBrepBody&,
                                                       const SCGeometryTolerance3d&, bool bounded);
    [[nodiscard]] PolyhedronBodyCandidate SolveBodyProjection(const SCPoint3d&, const PolyhedronBody&,
                                                             const SCGeometryTolerance3d&, bool bounded);
}
