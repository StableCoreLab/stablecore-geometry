#include "Core/Measure.h"
#include "Support/StableNumerics.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "Brep/MeshConversion.h"
#include "Brep/MeshOps.h"
#include "Brep/PolyhedronBody.h"
#include "Brep/PolyhedronFace3d.h"
#include "Brep/SCBrepBody.h"
#include "Brep/SCBrepEdge.h"
#include "Brep/SCBrepFace.h"
#include "Brep/SCBrepVertex.h"
#include "Core/Projection.h"
#include "../Detail/SurfaceProjectionSolver.h"
#include "../Detail/ProjectionCandidates.h"
#include "../Detail/BrepFaceRegion.h"
#include "../Detail/CurveOnSurfaceNumerics.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Geometry3d/SCNurbsSurface.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Core/ShapeOps.h"
#include "Geometry3d/ISCCurve3d.h"
#include "Geometry3d/ISCSurface.h"
#include "Geometry3d/Detail/SurfaceNumerics.h"
#include "Geometry3d/TriangleMesh.h"

namespace Geometry
{
    namespace
    {
        struct PointDistanceCandidate
        {
            double distance{0.0};
            double distanceSquared{0.0};
            bool validInput{false};
            bool distanceValid{false};
            bool distanceSquaredValid{false};
        };

        [[nodiscard]] PointDistanceCandidate EvaluatePointDistance(const SCPoint3d& lhs, const SCPoint3d& rhs)
        {
            PointDistanceCandidate candidate{};
            if (!lhs.IsValid() || !rhs.IsValid())
            {
                return candidate;
            }
            candidate.validInput = true;
            candidate.distance = std::hypot(rhs.x - lhs.x, rhs.y - lhs.y, rhs.z - lhs.z);
            candidate.distanceValid = std::isfinite(candidate.distance);
            if (candidate.distanceValid)
            {
                candidate.distanceSquared = candidate.distance * candidate.distance;
                candidate.distanceSquaredValid = std::isfinite(candidate.distanceSquared);
            }
            return candidate;
        }

        [[nodiscard]] PointDistanceCandidate EvaluateLineDistance(
            const SCPoint3d& point, const SCLine3d& line, const SCGeometryTolerance3d& tolerance)
        {
            if (!tolerance.IsValid() || !point.IsValid() || !line.IsValid(tolerance.distanceEpsilon))
            {
                return {};
            }
            PointDistanceCandidate candidate{};
            candidate.validInput = true;
            SCVector3d unitDirection;
            if (!Detail::TryUnitVector(line.direction, unitDirection))
            {
                return candidate;
            }
            const SCVector3d delta = point - line.origin;
            if (!delta.IsValid())
            {
                return candidate;
            }
            const double along = Dot(delta, unitDirection);
            if (!std::isfinite(along))
            {
                return candidate;
            }
            const SCVector3d residual = delta - unitDirection * along;
            if (!residual.IsValid())
            {
                return candidate;
            }
            candidate.distance = std::hypot(residual.x, residual.y, residual.z);
            candidate.distanceValid = std::isfinite(candidate.distance);
            if (candidate.distanceValid)
            {
                candidate.distanceSquared = candidate.distance * candidate.distance;
                candidate.distanceSquaredValid = std::isfinite(candidate.distanceSquared);
            }
            return candidate;
        }

        [[nodiscard]] PointDistanceCandidate EvaluatePlaneDistance(
            const SCPoint3d& point, const SCPlane& plane, const SCGeometryTolerance3d& tolerance)
        {
            if (!tolerance.IsValid() || !point.IsValid() || !plane.IsValid(tolerance.distanceEpsilon))
            {
                return {};
            }
            PointDistanceCandidate candidate{};
            candidate.validInput = true;
            SCVector3d unitNormal;
            if (!Detail::TryUnitVector(plane.normal, unitNormal))
            {
                return candidate;
            }
            candidate.distance = std::abs(plane.SignedDistanceTo(point,tolerance.distanceEpsilon));
            candidate.distanceValid = std::isfinite(candidate.distance);
            if (candidate.distanceValid)
            {
                candidate.distanceSquared = candidate.distance * candidate.distance;
                candidate.distanceSquaredValid = std::isfinite(candidate.distanceSquared);
            }
            return candidate;
        }

        [[nodiscard]] SCDistanceResult3dWithStatus PointDistanceWithStatus(
            const PointDistanceCandidate& candidate, bool requireSquared)
        {
            SCDistanceResult3dWithStatus result{};
            if (!candidate.validInput)
            {
                result.evidence.reasons = SCGeometryFailureReason::InvalidInput |
                                          SCGeometryFailureReason::NonFinite;
                return result;
            }
            if (!candidate.distanceValid || (requireSquared && !candidate.distanceSquaredValid))
            {
                result.state = SCGeometryOperationState::Unknown;
                result.evidence.reasons = SCGeometryFailureReason::Unrepresentable;
                return result;
            }
            result.state = SCGeometryOperationState::HasResult;
            result.payloadValid = true;
            result.payload.distance = candidate.distance;
            result.payload.distanceValid = true;
            if (candidate.distanceSquaredValid)
            {
                result.payload.distanceSquared = candidate.distanceSquared;
                result.payload.distanceSquaredValid = true;
            }
            result.evidence.optimality = SCGeometryEvidenceState::Verified;
            result.evidence.reasons = SCGeometryFailureReason::None;
            return result;
        }

        template <typename Projection>
        [[nodiscard]] PointDistanceCandidate EvaluateProjectionDistance(
            const SCPoint3d& point, const Projection& projection)
        {
            if (!projection.success || !projection.point.IsValid())
            {
                return {};
            }
            return EvaluatePointDistance(point, projection.point);
        }

        template <typename BodyProjection>
        [[nodiscard]] PointDistanceCandidate EvaluateBodyProjectionDistance(
            const SCPoint3d& point, const BodyProjection& projection)
        {
            if (!projection.success || !projection.projection.success || !projection.IsValid())
            {
                return {};
            }
            return EvaluatePointDistance(point, projection.projection.point);
        }

        [[nodiscard]] double PublicDistance(const PointDistanceCandidate& candidate, bool requireSquared)
        {
            if (requireSquared)
            {
                return candidate.distanceSquaredValid ? candidate.distanceSquared :
                    std::numeric_limits<double>::infinity();
            }
            return candidate.distanceValid ? candidate.distance : std::numeric_limits<double>::infinity();
        }

        template <typename Projection>
        SCDistanceResult3dWithStatus ProjectionDistanceStatus(const SCPoint3d& point,
            const Projection& candidate, bool squared, bool analytic, bool region = false,
            const SCGeometryEvidenceStatus* searchEvidence = nullptr)
        {
            auto result=PointDistanceWithStatus(EvaluateProjectionDistance(point,candidate),squared);
            if (!candidate.success) {
                result.state=SCGeometryOperationState::Unknown;
                result.evidence.reasons=SCGeometryFailureReason::EvaluationUnavailable;
            }
            result.evidence.region=region?SCGeometryEvidenceState::Verified:SCGeometryEvidenceState::NotApplicable;
            if (candidate.success) result.evidence.hit=SCGeometryEvidenceState::Verified;
            if (!analytic || !candidate.success) {
                result.evidence.optimality=SCGeometryEvidenceState::Unknown;
                result.evidence.reasons|=SCGeometryFailureReason::NoProof;
            }
            if (searchEvidence!=nullptr) {
                result.evidence.reasons|=searchEvidence->reasons;
                if (searchEvidence->optimality==SCGeometryEvidenceState::Unknown)
                    result.evidence.optimality=SCGeometryEvidenceState::Unknown;
            }
            return result;
        }
    }

    double DistanceSquared(const SCPoint3d& lhs, const SCPoint3d& rhs)
    {
        const PointDistanceCandidate candidate = EvaluatePointDistance(lhs, rhs);
        return candidate.distanceSquaredValid ? candidate.distanceSquared : std::numeric_limits<double>::infinity();
    }

    double Distance(const SCPoint3d& lhs, const SCPoint3d& rhs)
    {
        const PointDistanceCandidate candidate = EvaluatePointDistance(lhs, rhs);
        return candidate.distanceValid ? candidate.distance : std::numeric_limits<double>::infinity();
    }

    SCDistanceResult3dWithStatus DistanceSquaredWithStatus(const SCPoint3d& lhs, const SCPoint3d& rhs)
    {
        return PointDistanceWithStatus(EvaluatePointDistance(lhs, rhs), true);
    }

    SCDistanceResult3dWithStatus DistanceWithStatus(const SCPoint3d& lhs, const SCPoint3d& rhs)
    {
        return PointDistanceWithStatus(EvaluatePointDistance(lhs, rhs), false);
    }

    double DistanceSquared(const SCPoint3d& point, const SCLine3d& line, const SCGeometryTolerance3d& tolerance)
    {
        return PublicDistance(EvaluateLineDistance(point, line, tolerance), true);
    }

    double Distance(const SCPoint3d& point, const SCLine3d& line, const SCGeometryTolerance3d& tolerance)
    {
        return PublicDistance(EvaluateLineDistance(point, line, tolerance), false);
    }

    SCDistanceResult3dWithStatus DistanceSquaredWithStatus(
        const SCPoint3d& point, const SCLine3d& line, const SCGeometryTolerance3d& tolerance)
    {
        return PointDistanceWithStatus(EvaluateLineDistance(point, line, tolerance), true);
    }

    SCDistanceResult3dWithStatus DistanceWithStatus(
        const SCPoint3d& point, const SCLine3d& line, const SCGeometryTolerance3d& tolerance)
    {
        return PointDistanceWithStatus(EvaluateLineDistance(point, line, tolerance), false);
    }

    double DistanceSquared(const SCPoint3d& point, const SCPlane& plane)
    {
        return PublicDistance(EvaluatePlaneDistance(point, plane, {}), true);
    }

    double Distance(const SCPoint3d& point, const SCPlane& plane)
    {
        return PublicDistance(EvaluatePlaneDistance(point, plane, {}), false);
    }

    SCDistanceResult3dWithStatus DistanceSquaredWithStatus(
        const SCPoint3d& point, const SCPlane& plane, const SCGeometryTolerance3d& tolerance)
    {
        return PointDistanceWithStatus(EvaluatePlaneDistance(point, plane, tolerance), true);
    }

    SCDistanceResult3dWithStatus DistanceWithStatus(
        const SCPoint3d& point, const SCPlane& plane, const SCGeometryTolerance3d& tolerance)
    {
        return PointDistanceWithStatus(EvaluatePlaneDistance(point, plane, tolerance), false);
    }

    double DistanceSquared(const SCPoint3d& point, const ISCCurve3d& curve, const SCGeometryTolerance3d& tolerance)
    {
        return PublicDistance(EvaluateProjectionDistance(point, Detail::SolveCurveProjection(point, curve, tolerance)), true);
    }

    double Distance(const SCPoint3d& point, const ISCCurve3d& curve, const SCGeometryTolerance3d& tolerance)
    {
        return PublicDistance(EvaluateProjectionDistance(point, Detail::SolveCurveProjection(point, curve, tolerance)), false);
    }

    double DistanceSquared(const SCPoint3d& point,
                           const SCCurveOnSurface& curveOnSurface,
                           const SCGeometryTolerance3d& tolerance)
    {
        return PublicDistance(EvaluateProjectionDistance(
            point, Detail::SolveCurveOnSurfaceProjection(point, curveOnSurface, tolerance)), true);
    }

    double Distance(const SCPoint3d& point, const SCCurveOnSurface& curveOnSurface, const SCGeometryTolerance3d& tolerance)
    {
        return PublicDistance(EvaluateProjectionDistance(
            point, Detail::SolveCurveOnSurfaceProjection(point, curveOnSurface, tolerance)), false);
    }

    double DistanceSquared(const SCPoint3d& point, const ISCSurface& surface, const SCGeometryTolerance3d& tolerance)
    {
        Detail::SurfaceSearchBudget budget{};
        const auto candidate = Detail::SolveSurfaceProjection(point, surface, tolerance, budget);
        return candidate.success ? PublicDistance(EvaluatePointDistance(point, candidate.point), true) :
            std::numeric_limits<double>::infinity();
    }

    double Distance(const SCPoint3d& point, const ISCSurface& surface, const SCGeometryTolerance3d& tolerance)
    {
        Detail::SurfaceSearchBudget budget{};
        const auto candidate = Detail::SolveSurfaceProjection(point, surface, tolerance, budget);
        return candidate.success ? PublicDistance(EvaluatePointDistance(point, candidate.point), false) :
            std::numeric_limits<double>::infinity();
    }

    double DistanceSquared(const SCPoint3d& point, const SCBrepFace& face, const SCGeometryTolerance3d& tolerance)
    {
        return PublicDistance(EvaluateProjectionDistance(point, Detail::SolveFaceProjection(point, face, tolerance)), true);
    }

    double Distance(const SCPoint3d& point, const SCBrepFace& face, const SCGeometryTolerance3d& tolerance)
    {
        return PublicDistance(EvaluateProjectionDistance(point, Detail::SolveFaceProjection(point, face, tolerance)), false);
    }

    double DistanceSquared(const SCPoint3d& point, const SCBrepEdge& edge, const SCGeometryTolerance3d& tolerance)
    {
        return PublicDistance(EvaluateProjectionDistance(point, Detail::SolveEdgeProjection(point, edge, tolerance)), true);
    }

    double Distance(const SCPoint3d& point, const SCBrepEdge& edge, const SCGeometryTolerance3d& tolerance)
    {
        return PublicDistance(EvaluateProjectionDistance(point, Detail::SolveEdgeProjection(point, edge, tolerance)), false);
    }

    double DistanceSquared(const SCPoint3d& point, const SCBrepVertex& vertex, const SCGeometryTolerance3d& tolerance)
    {
        return PublicDistance(tolerance.IsValid() && vertex.IsValid()
            ? EvaluatePointDistance(point, vertex.Point()) : PointDistanceCandidate{}, true);
    }

    double Distance(const SCPoint3d& point, const SCBrepVertex& vertex, const SCGeometryTolerance3d& tolerance)
    {
        return PublicDistance(tolerance.IsValid() && vertex.IsValid()
            ? EvaluatePointDistance(point, vertex.Point()) : PointDistanceCandidate{}, false);
    }

    SCDistanceResult3dWithStatus DistanceSquaredWithStatus(
        const SCPoint3d& point, const SCBrepVertex& vertex, const SCGeometryTolerance3d& tolerance)
    {
        return PointDistanceWithStatus(tolerance.IsValid() && vertex.IsValid()
            ? EvaluatePointDistance(point, vertex.Point()) : PointDistanceCandidate{}, true);
    }

    SCDistanceResult3dWithStatus DistanceWithStatus(
        const SCPoint3d& point, const SCBrepVertex& vertex, const SCGeometryTolerance3d& tolerance)
    {
        return PointDistanceWithStatus(tolerance.IsValid() && vertex.IsValid()
            ? EvaluatePointDistance(point, vertex.Point()) : PointDistanceCandidate{}, false);
    }

    double DistanceSquared(const SCPoint3d& point, const SCBrepBody& body, const SCGeometryTolerance3d& tolerance)
    {
        const auto result=Detail::SolveBodyProjection(point,body,tolerance,false);
        return result.validInput && result.allFacesAvailable && result.candidate.success
            ? PublicDistance(EvaluatePointDistance(point,result.candidate.projection.point),true)
            : std::numeric_limits<double>::infinity();
    }

    double Distance(const SCPoint3d& point, const SCBrepBody& body, const SCGeometryTolerance3d& tolerance)
    {
        const auto result=Detail::SolveBodyProjection(point,body,tolerance,false);
        return result.validInput && result.allFacesAvailable && result.candidate.success
            ? PublicDistance(EvaluatePointDistance(point,result.candidate.projection.point),false)
            : std::numeric_limits<double>::infinity();
    }

    double DistanceSquared(const SCPoint3d& point, const PolyhedronFace3d& face, const SCGeometryTolerance3d& tolerance)
    {
        return PublicDistance(EvaluateProjectionDistance(point, Detail::SolvePolyFaceProjection(point, face, tolerance)), true);
    }

    double Distance(const SCPoint3d& point, const PolyhedronFace3d& face, const SCGeometryTolerance3d& tolerance)
    {
        return PublicDistance(EvaluateProjectionDistance(point, Detail::SolvePolyFaceProjection(point, face, tolerance)), false);
    }

    double DistanceSquared(const SCPoint3d& point, const PolyhedronBody& body, const SCGeometryTolerance3d& tolerance)
    {
        const auto result=Detail::SolveBodyProjection(point,body,tolerance,false);
        return result.validInput && result.allFacesAvailable && result.candidate.success
            ? PublicDistance(EvaluatePointDistance(point,result.candidate.projection.point),true)
            : std::numeric_limits<double>::infinity();
    }

    double Distance(const SCPoint3d& point, const PolyhedronBody& body, const SCGeometryTolerance3d& tolerance)
    {
        const auto result=Detail::SolveBodyProjection(point,body,tolerance,false);
        return result.validInput && result.allFacesAvailable && result.candidate.success
            ? PublicDistance(EvaluatePointDistance(point,result.candidate.projection.point),false)
            : std::numeric_limits<double>::infinity();
    }

    double DistanceSquared(const SCPoint3d& point, const TriangleMesh& mesh, const SCGeometryTolerance3d& tolerance)
    {
        return PublicDistance(EvaluateProjectionDistance(point, Detail::SolveMeshProjection(point, mesh, tolerance)), true);
    }

    double Distance(const SCPoint3d& point, const TriangleMesh& mesh, const SCGeometryTolerance3d& tolerance)
    {
        return PublicDistance(EvaluateProjectionDistance(point, Detail::SolveMeshProjection(point, mesh, tolerance)), false);
    }

    namespace
    {
        SCDistanceResult3dWithStatus SurfaceDistanceStatus(const SCPoint3d& point, const ISCSurface& surface,
            const SCGeometryTolerance3d& tolerance, bool squared)
        {
            if (!tolerance.IsValid() || !point.IsValid() || !surface.IsValid(tolerance)) return {};
            Detail::SurfaceSearchBudget budget{};
            budget.bounded = dynamic_cast<const SCNurbsSurface*>(&surface) != nullptr;
            const auto candidate = Detail::SolveSurfaceProjection(point, surface, tolerance, budget);
            auto result = candidate.success ? PointDistanceWithStatus(EvaluatePointDistance(point, candidate.point), squared)
                                           : SCDistanceResult3dWithStatus{};
            if (!candidate.success) {
                result.state = SCGeometryOperationState::Unknown;
                result.evidence.reasons = SCGeometryFailureReason::EvaluationUnavailable;
            }
            if (candidate.success) result.evidence.hit = SCGeometryEvidenceState::Verified;
            if (dynamic_cast<const SCPlaneSurface*>(&surface) == nullptr) {
                result.evidence.optimality = SCGeometryEvidenceState::Unknown;
                result.evidence.reasons |= SCGeometryFailureReason::NoProof;
            }
            if (budget.statistics.searchIncomplete) {
                result.evidence.optimality = SCGeometryEvidenceState::Unknown;
                result.evidence.reasons |= SCGeometryFailureReason::SearchIncomplete;
            }
            return result;
        }
    }

    SCDistanceResult3dWithStatus DistanceWithStatus(const SCPoint3d& point, const ISCSurface& target,
        const SCGeometryTolerance3d& tolerance) { return SurfaceDistanceStatus(point, target, tolerance, false); }
    SCDistanceResult3dWithStatus DistanceSquaredWithStatus(const SCPoint3d& point, const ISCSurface& target,
        const SCGeometryTolerance3d& tolerance) { return SurfaceDistanceStatus(point, target, tolerance, true); }

    SCDistanceResult3dWithStatus DistanceWithStatus(const SCPoint3d& point, const ISCCurve3d& target,
        const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !point.IsValid() || !target.IsValid(tolerance)) return {};
        const auto candidate=Detail::SolveCurveProjection(point,target,tolerance);
        return ProjectionDistanceStatus(point,candidate,false,dynamic_cast<const SCLineCurve3d*>(&target)!=nullptr,false);
    }

    SCDistanceResult3dWithStatus DistanceSquaredWithStatus(const SCPoint3d& point, const ISCCurve3d& target,
        const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !point.IsValid() || !target.IsValid(tolerance)) return {};
        const auto candidate=Detail::SolveCurveProjection(point,target,tolerance);
        return ProjectionDistanceStatus(point,candidate,true,dynamic_cast<const SCLineCurve3d*>(&target)!=nullptr,false);
    }

    SCDistanceResult3dWithStatus DistanceWithStatus(const SCPoint3d& point, const SCCurveOnSurface& target,
        const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !point.IsValid() || !target.IsValid(tolerance)) return {};
        bool analyticComplete=false;
        const auto candidate=Detail::SolveCurveOnSurfaceProjection(point,target,tolerance,&analyticComplete);
        return ProjectionDistanceStatus(point,candidate,false,analyticComplete,false);
    }

    SCDistanceResult3dWithStatus DistanceSquaredWithStatus(const SCPoint3d& point, const SCCurveOnSurface& target,
        const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !point.IsValid() || !target.IsValid(tolerance)) return {};
        bool analyticComplete=false;
        const auto candidate=Detail::SolveCurveOnSurfaceProjection(point,target,tolerance,&analyticComplete);
        return ProjectionDistanceStatus(point,candidate,true,analyticComplete,false);
    }

    SCDistanceResult3dWithStatus DistanceWithStatus(const SCPoint3d& point, const SCBrepFace& target,
        const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !point.IsValid() || !target.IsValid(tolerance)) return {};
        SCPolygon2d polygon;
        if (!Detail::TryBuildPolygonFromBrepFaceTrims(target,tolerance,polygon)) {
            SCDistanceResult3dWithStatus result{};
            result.state=SCGeometryOperationState::Unknown;
            result.evidence.region=SCGeometryEvidenceState::Unknown;
            result.evidence.optimality=SCGeometryEvidenceState::Unknown;
            result.evidence.reasons=SCGeometryFailureReason::RegionUnavailable;
            return result;
        }
        SCGeometryEvidenceStatus searchEvidence{};
        searchEvidence.reasons=SCGeometryFailureReason::None;
        const auto candidate=Detail::SolveFaceProjection(point,target,tolerance,true,&searchEvidence);
        return ProjectionDistanceStatus(point,candidate,false,dynamic_cast<const SCPlaneSurface*>(target.SupportSurface())!=nullptr,true,&searchEvidence);
    }

    SCDistanceResult3dWithStatus DistanceSquaredWithStatus(const SCPoint3d& point, const SCBrepFace& target,
        const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !point.IsValid() || !target.IsValid(tolerance)) return {};
        SCPolygon2d polygon;
        if (!Detail::TryBuildPolygonFromBrepFaceTrims(target,tolerance,polygon)) {
            SCDistanceResult3dWithStatus result{};
            result.state=SCGeometryOperationState::Unknown;
            result.evidence.region=SCGeometryEvidenceState::Unknown;
            result.evidence.optimality=SCGeometryEvidenceState::Unknown;
            result.evidence.reasons=SCGeometryFailureReason::RegionUnavailable;
            return result;
        }
        SCGeometryEvidenceStatus searchEvidence{};
        searchEvidence.reasons=SCGeometryFailureReason::None;
        const auto candidate=Detail::SolveFaceProjection(point,target,tolerance,true,&searchEvidence);
        return ProjectionDistanceStatus(point,candidate,true,dynamic_cast<const SCPlaneSurface*>(target.SupportSurface())!=nullptr,true,&searchEvidence);
    }

    SCDistanceResult3dWithStatus DistanceWithStatus(const SCPoint3d& point, const SCBrepEdge& target,
        const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !point.IsValid() || !target.IsValid(tolerance)) return {};
        const auto candidate=Detail::SolveEdgeProjection(point,target,tolerance);
        return ProjectionDistanceStatus(point,candidate,false,dynamic_cast<const SCLineCurve3d*>(target.Curve())!=nullptr,false);
    }

    SCDistanceResult3dWithStatus DistanceSquaredWithStatus(const SCPoint3d& point, const SCBrepEdge& target,
        const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !point.IsValid() || !target.IsValid(tolerance)) return {};
        const auto candidate=Detail::SolveEdgeProjection(point,target,tolerance);
        return ProjectionDistanceStatus(point,candidate,true,dynamic_cast<const SCLineCurve3d*>(target.Curve())!=nullptr,false);
    }

    SCDistanceResult3dWithStatus DistanceWithStatus(const SCPoint3d& point, const PolyhedronFace3d& target,
        const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !point.IsValid() || !target.IsValid(tolerance)) return {};
        const auto candidate=Detail::SolvePolyFaceProjection(point,target,tolerance);
        return ProjectionDistanceStatus(point,candidate,false,true,true);
    }

    SCDistanceResult3dWithStatus DistanceSquaredWithStatus(const SCPoint3d& point, const PolyhedronFace3d& target,
        const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !point.IsValid() || !target.IsValid(tolerance)) return {};
        const auto candidate=Detail::SolvePolyFaceProjection(point,target,tolerance);
        return ProjectionDistanceStatus(point,candidate,true,true,true);
    }

    SCDistanceResult3dWithStatus DistanceWithStatus(const SCPoint3d& point, const TriangleMesh& target,
        const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !point.IsValid() || !target.IsValid(tolerance.distanceEpsilon)) return {};
        const auto candidate=Detail::SolveMeshProjection(point,target,tolerance);
        return ProjectionDistanceStatus(point,candidate,false,true,true);
    }

    SCDistanceResult3dWithStatus DistanceSquaredWithStatus(const SCPoint3d& point, const TriangleMesh& target,
        const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !point.IsValid() || !target.IsValid(tolerance.distanceEpsilon)) return {};
        const auto candidate=Detail::SolveMeshProjection(point,target,tolerance);
        return ProjectionDistanceStatus(point,candidate,true,true,true);
    }

    SCDistanceResult3dWithStatus DistanceWithStatus(const SCPoint3d& point, const SCBrepBody& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto internal=Detail::SolveBodyProjection(point,target,tolerance,true);
        if (!internal.validInput) return {};
        SCDistanceResult3dWithStatus result{};
        if (internal.candidate.success)
            result=PointDistanceWithStatus(EvaluatePointDistance(point,internal.candidate.projection.point),false);
        else { result.state=SCGeometryOperationState::Unknown; }
        const auto distanceReasons=result.evidence.reasons;
        result.evidence=internal.evidence;
        if (!result.payloadValid) result.evidence.reasons|=internal.candidate.success?distanceReasons:SCGeometryFailureReason::EvaluationUnavailable;
        return result;
    }

    SCDistanceResult3dWithStatus DistanceSquaredWithStatus(const SCPoint3d& point, const SCBrepBody& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto internal=Detail::SolveBodyProjection(point,target,tolerance,true);
        if (!internal.validInput) return {};
        SCDistanceResult3dWithStatus result{};
        if (internal.candidate.success)
            result=PointDistanceWithStatus(EvaluatePointDistance(point,internal.candidate.projection.point),true);
        else { result.state=SCGeometryOperationState::Unknown; }
        const auto distanceReasons=result.evidence.reasons;
        result.evidence=internal.evidence;
        if (!result.payloadValid) result.evidence.reasons|=internal.candidate.success?distanceReasons:SCGeometryFailureReason::EvaluationUnavailable;
        return result;
    }

    SCDistanceResult3dWithStatus DistanceWithStatus(const SCPoint3d& point, const PolyhedronBody& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto internal=Detail::SolveBodyProjection(point,target,tolerance,true);
        if (!internal.validInput) return {};
        SCDistanceResult3dWithStatus result{};
        if (internal.candidate.success)
            result=PointDistanceWithStatus(EvaluatePointDistance(point,internal.candidate.projection.point),false);
        else { result.state=SCGeometryOperationState::Unknown; }
        const auto distanceReasons=result.evidence.reasons;
        result.evidence=internal.evidence;
        if (!result.payloadValid) result.evidence.reasons|=internal.candidate.success?distanceReasons:SCGeometryFailureReason::EvaluationUnavailable;
        return result;
    }

    SCDistanceResult3dWithStatus DistanceSquaredWithStatus(const SCPoint3d& point, const PolyhedronBody& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto internal=Detail::SolveBodyProjection(point,target,tolerance,true);
        if (!internal.validInput) return {};
        SCDistanceResult3dWithStatus result{};
        if (internal.candidate.success)
            result=PointDistanceWithStatus(EvaluatePointDistance(point,internal.candidate.projection.point),true);
        else { result.state=SCGeometryOperationState::Unknown; }
        const auto distanceReasons=result.evidence.reasons;
        result.evidence=internal.evidence;
        if (!result.payloadValid) result.evidence.reasons|=internal.candidate.success?distanceReasons:SCGeometryFailureReason::EvaluationUnavailable;
        return result;
    }

    double Length(const SCLineSegment3d& segment)
    {
        return Distance(segment.startPoint,segment.endPoint);
    }

    double Length(const ISCCurve3d& curve, std::size_t sampleCount)
    {
        const SCIntervald range = curve.ParameterRange();
        if (!range.IsValid() || sampleCount == 0)
        {
            return 0.0;
        }

        const std::size_t steps = std::max<std::size_t>(sampleCount, 1);
        double total = 0.0;
        SCPoint3d previous = curve.PointAt(range.min);
        if (!previous.IsValid()) return std::numeric_limits<double>::infinity();
        for (std::size_t i = 1; i <= steps; ++i)
        {
            const auto parameter=Detail::StableLerp(range.min,range.max,static_cast<double>(i)/static_cast<double>(steps));
            if (!parameter) return std::numeric_limits<double>::infinity();
            const SCPoint3d current = curve.PointAt(*parameter);
            total += Distance(previous, current);
            previous = current;
        }
        return total;
    }

    double Length(const SCCurveOnSurface& curveOnSurface)
    {
        if (!curveOnSurface.IsValid() || curveOnSurface.PointCount() < 2)
        {
            return 0.0;
        }

        double total = 0.0;
        const std::size_t segmentCount=curveOnSurface.UvCurve().IsClosed()?curveOnSurface.PointCount():curveOnSurface.PointCount()-1;
        for (std::size_t segment=0;segment<segmentCount;++segment) {
            const auto length=[&](auto&& self,double first,double last,const SCPoint3d& a,const SCPoint3d& b,int depth)->double {
                const double middle=std::lerp(first,last,0.5);
                const auto sample=Detail::EvaluateCurveOnSurfaceSegment(curveOnSurface,segment,middle);
                if (!sample.valid) return std::numeric_limits<double>::infinity();
                const double chord=Distance(a,b);
                const double split=Distance(a,sample.point)+Distance(sample.point,b);
                if (!std::isfinite(chord) || !std::isfinite(split)) return std::numeric_limits<double>::infinity();
                const double allowance=std::max(kDefaultEpsilon*(last-first),
                    64*std::numeric_limits<double>::epsilon()*split);
                if (depth==8 || split-chord<=allowance || middle==first || middle==last) return split;
                return self(self,first,middle,a,sample.point,depth+1)+self(self,middle,last,sample.point,b,depth+1);
            };
            constexpr std::size_t samples=32;
            auto previous=Detail::EvaluateCurveOnSurfaceSegment(curveOnSurface,segment,0);
            if (!previous.valid) return std::numeric_limits<double>::infinity();
            for (std::size_t i=1;i<=samples;++i) {
                const double first=static_cast<double>(i-1)/samples,last=static_cast<double>(i)/samples;
                const auto current=Detail::EvaluateCurveOnSurfaceSegment(curveOnSurface,segment,last);
                if (!current.valid) return std::numeric_limits<double>::infinity();
                total+=length(length,first,last,previous.point,current.point,0);
                if (!std::isfinite(total)) return std::numeric_limits<double>::infinity();
                previous=current;
            }
        }
        return total;
    }

    double Area(const SCTriangle3d& triangle)
    {
        return triangle.Area();
    }

    double Area(const PolyhedronFace3d& face, const SCGeometryTolerance3d& tolerance)
    {
        const SCFaceProjection3d projection = ProjectFaceToPolygon2d(face, tolerance);
        if (!projection.success)
        {
            return 0.0;
        }

        return std::abs(projection.polygon.Area());
    }

    double Area(const SCBrepFace& face, const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid())
        {
            return 0.0;
        }
        const PolyhedronMeshConversion3d conversion = ConvertToTriangleMesh(face, tolerance);
        if (!conversion.success)
        {
            return 0.0;
        }

        return conversion.mesh.SurfaceArea();
    }

    double Area(const SCBrepFace& face, double eps)
    {
        return Area(face, SCGeometryTolerance3d{eps, eps, eps, eps});
    }

    double Area(const TriangleMesh& mesh)
    {
        return mesh.SurfaceArea();
    }

    double Volume(const TriangleMesh& mesh)
    {
        if (!mesh.IsValid() || !IsClosedTriangleMesh(mesh))
        {
            return 0.0;
        }

        double signedVolume = 0.0;
        for (std::size_t i = 0; i < mesh.TriangleCount(); ++i)
        {
            const SCTriangle3d triangle = mesh.TriangleAt(i);
            signedVolume += Dot(triangle.a - SCPoint3d{}, Cross(triangle.b - SCPoint3d{}, triangle.c - SCPoint3d{})) / 6.0;
        }
        return std::abs(signedVolume);
    }

    double Volume(const PolyhedronBody& body, const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid())
        {
            return 0.0;
        }
        const PolyhedronMeshConversion3d conversion = ConvertToTriangleMesh(body, tolerance.distanceEpsilon);
        if (!conversion.success)
        {
            return 0.0;
        }

        return Volume(conversion.mesh);
    }

    double Volume(const SCBrepBody& body, const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid())
        {
            return 0.0;
        }
        const PolyhedronMeshConversion3d conversion = ConvertToTriangleMesh(body, tolerance);
        if (!conversion.success)
        {
            return 0.0;
        }

        return Volume(conversion.mesh);
    }

    double Volume(const PolyhedronBody& body, double eps)
    {
        return Volume(body, SCGeometryTolerance3d{eps, eps, eps, eps});
    }

    double Volume(const SCBrepBody& body, double eps)
    {
        return Volume(body, SCGeometryTolerance3d{eps, eps, eps, eps});
    }

    SCBox3d Bounds(const ISCCurve3d& curve)
    {
        return curve.Bounds();
    }

    SCBox3d Bounds(const ISCSurface& surface)
    {
        return surface.Bounds();
    }

    SCBox3d Bounds(const SCCurveOnSurface& curveOnSurface)
    {
        return curveOnSurface.Bounds();
    }

    SCBox3d Bounds(const TriangleMesh& mesh)
    {
        return mesh.Bounds();
    }

    SCBox3d Bounds(const PolyhedronFace3d& face)
    {
        return face.Bounds();
    }

    SCBox3d Bounds(const PolyhedronBody& body)
    {
        return body.Bounds();
    }

    SCBox3d Bounds(const SCBrepVertex& vertex)
    {
        if (!vertex.IsValid())
        {
            return {};
        }

        return SCBox3d::FromMinMax(vertex.Point(), vertex.Point());
    }

    SCBox3d Bounds(const SCBrepEdge& edge)
    {
        return edge.Bounds();
    }

    SCBox3d Bounds(const SCBrepFace& face)
    {
        return face.Bounds();
    }

    SCBox3d Bounds(const SCBrepBody& body)
    {
        return body.Bounds();
    }
}  // namespace Geometry

