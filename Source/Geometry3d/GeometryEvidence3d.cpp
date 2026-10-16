#include "Core/Intersection.h"
#include "Core/Projection.h"
#include "Core/Measure.h"
#include "Core/Relation.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <set>

#include "Brep/SCBrepBody.h"
#include "Brep/SCBrepEdge.h"
#include "Brep/SCBrepVertex.h"
#include "Brep/PolyhedronBody.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Geometry3d/TriangleMesh.h"
#include "../Detail/BrepFaceRegion.h"
#include "../Detail/LineFaceHits.h"

namespace Geometry
{
    namespace
    {
        using State = SCGeometryOperationState;
        using Evidence = SCGeometryEvidenceState;
        using Reason = SCGeometryFailureReason;

        template <typename Result>
        void Unknown(Result& result, Reason reason)
        {
            result.state = State::Unknown;
            result.payloadValid = false;
            result.evidence.negativeCompleteness = Evidence::Unknown;
            result.evidence.reasons = reason;
        }

        bool Close(const SCPoint3d& a, const SCPoint3d& b, double epsilon)
        {
            return a.IsValid() && b.IsValid() &&
                std::hypot(a.x-b.x, a.y-b.y, a.z-b.z) <= epsilon;
        }

        template <typename Result, typename Payload>
        Result LineCandidate(
            const SCLine3d& line, Payload payload, const SCPoint3d& actual,
            const SCGeometryTolerance3d& tolerance, bool complete)
        {
            Result result{};
            Unknown(result, Reason::SearchIncomplete | Reason::NoProof);
            if (payload.intersects && payload.IsValid() && std::isfinite(payload.lineParameter) &&
                Detail::SameRoundedPosition(actual, payload.point) &&
                Close(line.PointAt(payload.lineParameter), actual, tolerance.distanceEpsilon)) {
                result.state = State::HasResult;
                result.payloadValid = true;
                result.payload = payload;
                result.evidence.hit = Evidence::Verified;
            }
            if (complete) {
                result.evidence.negativeCompleteness = Evidence::Verified;
                result.evidence.reasons = Reason::None;
                if (!result.payloadValid) result.state = State::NoResult;
            }
            return result;
        }

        struct PlanarHits
        {
            bool complete{false};
            bool continuous{false};
            std::vector<SCLineBrepFaceIntersection3d> hits{};
        };

        // 平面区域的离散交点与共面连续交集分开处理；UV 区域不受曲面矩形域钳制。
        PlanarHits CollectPlanarRegionHits(const SCLine3d& line, const SCPlaneSurface& surface,
            const SCPolygon2d& polygon, const SCGeometryTolerance3d& tolerance)
        {
            PlanarHits result{};
            SCPlane plane;
            SCVector3d direction;
            if (!Detail::TryActualPlane(surface, tolerance, plane) ||
                !Detail::TryUnitVector(line.direction, direction)) return result;
            const double length = std::hypot(line.direction.x, line.direction.y, line.direction.z);
            const double denominator = Dot(direction, plane.normal);
            const double height = Dot(line.origin-plane.origin, plane.normal);
            if (!std::isfinite(length) || !std::isfinite(denominator) || !std::isfinite(height)) return result;
            if (denominator != 0.0) {
                if (std::abs(denominator) <= tolerance.angleEpsilon) return result;
                const double t = (-height / denominator) / length;
                if (!std::isfinite(t)) return result;
                const auto point = line.PointAt(t);
                SCPoint2d uv;
                if (!Detail::TryAffinePlaneUv(surface, point, tolerance, uv)) return result;
                const auto actual = surface.PointAt(uv.x, uv.y);
                if (!Close(point, actual, tolerance.distanceEpsilon)) return result;
                const auto containment = LocatePoint(uv, polygon, tolerance.parameterEpsilon);
                result.complete = true;
                if (containment != SCPointContainment2d::Outside)
                    result.hits.push_back({true, containment == SCPointContainment2d::OnBoundary,
                                           t, uv.x, uv.y, actual});
                return result;
            }
            if (std::abs(height) > tolerance.distanceEpsilon) {
                result.complete = true;
                return result;
            }
            // 近似共面不足以证明连续多解。
            if (height != 0.0) return result;
            SCPoint2d uv0, uv1;
            if (!Detail::TryAffinePlaneUv(surface, line.origin, tolerance, uv0) ||
                !Detail::TryAffinePlaneUv(surface, line.origin + direction, tolerance, uv1)) return result;
            const auto delta = uv1-uv0;
            if (!delta.IsValid() || (delta.x == 0.0 && delta.y == 0.0)) return result;
            std::vector<double> events;
            const auto cross = [](const auto& a, const auto& b) { return a.x*b.y-a.y*b.x; };
            const auto appendRing = [&](const SCPolyline2d& ring) {
                for (std::size_t i=0; i<ring.PointCount(); ++i) {
                    const auto a = ring.PointAt(i);
                    const auto b = ring.PointAt((i+1)%ring.PointCount());
                    const auto edge = b-a;
                    const double det = cross(delta, edge);
                    if (!std::isfinite(det)) return false;
                    if (det == 0.0) {
                        if (cross(a-uv0, delta) == 0.0) {
                            const bool useX = std::abs(delta.x) >= std::abs(delta.y);
                            const double s0 = useX ? (a.x-uv0.x)/delta.x : (a.y-uv0.y)/delta.y;
                            const double s1 = useX ? (b.x-uv0.x)/delta.x : (b.y-uv0.y)/delta.y;
                            if (!std::isfinite(s0) || !std::isfinite(s1)) return false;
                            events.push_back(s0); events.push_back(s1);
                            if (s0 != s1) result.continuous = true;
                        }
                        continue;
                    }
                    const double s = cross(a-uv0, edge)/det;
                    const double f = cross(a-uv0, delta)/det;
                    if (!std::isfinite(s) || !std::isfinite(f)) return false;
                    if (f >= 0.0 && f <= 1.0) events.push_back(s);
                }
                return true;
            };
            if (!appendRing(polygon.OuterRing())) return {};
            for (std::size_t i=0; i<polygon.HoleCount(); ++i) if (!appendRing(polygon.HoleAt(i))) return {};
            std::sort(events.begin(), events.end());
            events.erase(std::unique(events.begin(), events.end()), events.end());
            std::vector<bool> attachedToInterval(events.size(),false);
            for (std::size_t i=1; i<events.size(); ++i) {
                const auto uv = uv0 + delta*std::lerp(events[i-1], events[i], 0.5);
                if (events[i-1] < events[i] && LocatePoint(uv, polygon, tolerance.parameterEpsilon) !=
                    SCPointContainment2d::Outside) {
                    result.continuous = true;
                    attachedToInterval[i-1]=attachedToInterval[i]=true;
                }
            }
            for (std::size_t i=0;i<events.size();++i) {
                if (attachedToInterval[i]) continue;
                const double s=events[i];
                const auto uv = uv0+delta*s;
                const double t = s/length;
                const auto actual = surface.PointAt(uv.x, uv.y);
                if (!std::isfinite(t) || !Close(actual, line.PointAt(t), tolerance.distanceEpsilon)) return {};
                if (LocatePoint(uv, polygon, tolerance.parameterEpsilon) != SCPointContainment2d::Outside)
                    result.hits.push_back({true, true, t, uv.x, uv.y, actual});
            }
            result.complete = true;
            return result;
        }
    }

    SCLinePlaneIntersection3dWithStatus IntersectLinePlaneWithStatus(
        const SCLine3d& line, const SCPlane& plane, const SCGeometryTolerance3d& tolerance)
    {
        SCLinePlaneIntersection3dWithStatus result{};
        if (!tolerance.IsValid() || !line.IsValid(tolerance.distanceEpsilon) || !plane.IsValid(tolerance.distanceEpsilon)) return result;
        Unknown(result,Reason::NoProof);
        const auto hit=Intersect(line,plane,tolerance);
        SCVector3d direction, normal;
        if (!Detail::TryUnitVector(line.direction,direction) || !Detail::TryUnitVector(plane.normal,normal)) return result;
        const double denominator=Dot(direction,normal);
        const double height=Dot(line.origin-plane.origin,normal);
        if (!std::isfinite(height) || !std::isfinite(denominator)) return result;
        if (denominator==0 && hit.liesOnPlane) {
            result.state=State::NonUnique; result.payload=hit;
            result.evidence.hit=Evidence::Verified;
            result.evidence.negativeCompleteness=Evidence::NotApplicable;
            result.evidence.reasons=Reason::NonUnique;
        } else if (denominator==0 && std::abs(height)>tolerance.distanceEpsilon) {
            result.state=State::NoResult;
            result.evidence.negativeCompleteness=Evidence::Verified;
            result.evidence.reasons=Reason::None;
        } else if (hit.intersects && std::isfinite(hit.parameter) && hit.point.IsValid() &&
                   Close(hit.point,line.PointAt(hit.parameter),tolerance.distanceEpsilon) &&
                   std::abs(plane.SignedDistanceTo(hit.point,tolerance.distanceEpsilon))<=tolerance.distanceEpsilon) {
            result.state=State::HasResult; result.payloadValid=true; result.payload=hit;
            result.evidence.hit=Evidence::Verified;
            result.evidence.negativeCompleteness=Evidence::Verified;
            result.evidence.reasons=Reason::None;
        }
        return result;
    }

    SCPlanePlaneIntersection3dWithStatus IntersectPlanePlaneWithStatus(
        const SCPlane& first, const SCPlane& second, const SCGeometryTolerance3d& tolerance)
    {
        SCPlanePlaneIntersection3dWithStatus result{};
        if (!tolerance.IsValid() || !first.IsValid(tolerance.distanceEpsilon) || !second.IsValid(tolerance.distanceEpsilon)) return result;
        Unknown(result,Reason::NoProof);
        SCVector3d a,b;
        if (!Detail::TryUnitVector(first.normal,a) || !Detail::TryUnitVector(second.normal,b)) return result;
        const auto cross=Cross(a,b);
        const double sine=std::hypot(cross.x,cross.y,cross.z);
        const double offset=Dot(second.origin-first.origin,b);
        if (!std::isfinite(offset) || !std::isfinite(sine)) return result;
        if (sine==0) {
            result.payload.isParallel=true;
            result.evidence.reasons=Reason::None;
            if (std::abs(offset)<=tolerance.distanceEpsilon) {
                result.state=State::NonUnique;
                result.payload.intersects=true; result.payload.isCoincident=true;
                result.evidence.hit=Evidence::Verified;
                result.evidence.negativeCompleteness=Evidence::NotApplicable;
                result.evidence.reasons=Reason::NonUnique;
            } else {
                result.state=State::NoResult;
                result.evidence.negativeCompleteness=Evidence::Verified;
            }
            return result;
        }
        if (sine<=tolerance.angleEpsilon) return result;
        const auto direction=cross/sine;
        const auto point=first.origin+Cross(direction,a)*(offset/sine);
        const double residualA=std::abs(first.SignedDistanceTo(point,tolerance.distanceEpsilon));
        const double residualB=std::abs(second.SignedDistanceTo(point,tolerance.distanceEpsilon));
        if (!point.IsValid() || !std::isfinite(residualA) || !std::isfinite(residualB) ||
            residualA>tolerance.distanceEpsilon || residualB>tolerance.distanceEpsilon) return result;
        result.state=State::HasResult; result.payloadValid=true;
        result.payload.intersects=true;
        result.payload.line=SCLine3d::FromOriginAndDirection(point,direction);
        result.evidence.hit=Evidence::Verified;
        result.evidence.negativeCompleteness=Evidence::Verified;
        result.evidence.reasons=Reason::None;
        return result;
    }

    SCLineBrepEdgeIntersection3dWithStatus IntersectLineBrepEdgeWithStatus(
        const SCLine3d& line, const SCBrepEdge& edge, const SCGeometryTolerance3d& tolerance)
    {
        SCLineBrepEdgeIntersection3dWithStatus result{};
        if (!tolerance.IsValid() || !line.IsValid(tolerance.distanceEpsilon) || !edge.IsValid(tolerance) || !edge.Curve()) return result;
        const auto curve=IntersectLineCurveWithStatus(line,*edge.Curve(),tolerance);
        result.state=curve.state; result.payloadValid=curve.payloadValid; result.evidence=curve.evidence;
        if (curve.payloadValid) result.payload={true,curve.payload.lineParameter,curve.payload.curveParameter,curve.payload.point};
        return result;
    }

    SCPlaneBrepEdgeIntersection3dWithStatus IntersectPlaneBrepEdgeWithStatus(
        const SCPlane& plane, const SCBrepEdge& edge, const SCGeometryTolerance3d& tolerance)
    {
        SCPlaneBrepEdgeIntersection3dWithStatus result{};
        if (!tolerance.IsValid() || !plane.IsValid(tolerance.distanceEpsilon) || !edge.IsValid(tolerance) || !edge.Curve()) return result;
        const auto curve=IntersectPlaneCurveWithStatus(plane,*edge.Curve(),tolerance);
        result.state=curve.state; result.payloadValid=curve.payloadValid; result.evidence=curve.evidence;
        if (curve.payloadValid) result.payload={true,curve.payload.curveParameter,curve.payload.point};
        return result;
    }

    SCLineBrepVertexIntersection3dWithStatus IntersectLineBrepVertexWithStatus(
        const SCLine3d& line, const SCBrepVertex& vertex, const SCGeometryTolerance3d& tolerance)
    {
        SCLineBrepVertexIntersection3dWithStatus result{};
        if (!tolerance.IsValid() || !line.IsValid(tolerance.distanceEpsilon) || !vertex.IsValid()) return result;
        Unknown(result,Reason::Unrepresentable);
        const double distance=Distance(vertex.Point(),line,tolerance);
        if (!std::isfinite(distance)) return result;
        if (distance>tolerance.distanceEpsilon) {
            result.state=State::NoResult;
            result.evidence.negativeCompleteness=Evidence::Verified;
            result.evidence.reasons=Reason::None;
            return result;
        }
        const auto projection=ProjectPointToLine(vertex.Point(),line,tolerance);
        if (!std::isfinite(projection.parameter) || !Close(vertex.Point(),line.PointAt(projection.parameter),tolerance.distanceEpsilon)) return result;
        result.state=State::HasResult; result.payloadValid=true;
        result.payload={true,projection.parameter,vertex.Point()};
        result.evidence.hit=Evidence::Verified;
        result.evidence.negativeCompleteness=Evidence::Verified;
        result.evidence.reasons=Reason::None;
        return result;
    }

    SCPlaneBrepVertexIntersection3dWithStatus IntersectPlaneBrepVertexWithStatus(
        const SCPlane& plane, const SCBrepVertex& vertex, const SCGeometryTolerance3d& tolerance)
    {
        SCPlaneBrepVertexIntersection3dWithStatus result{};
        if (!tolerance.IsValid() || !plane.IsValid(tolerance.distanceEpsilon) || !vertex.IsValid()) return result;
        Unknown(result,Reason::Unrepresentable);
        const double distance=std::abs(plane.SignedDistanceTo(vertex.Point(),tolerance.distanceEpsilon));
        if (!std::isfinite(distance)) return result;
        result.evidence.negativeCompleteness=Evidence::Verified;
        result.evidence.reasons=Reason::None;
        result.state=State::NoResult;
        if (distance<=tolerance.distanceEpsilon) {
            result.state=State::HasResult; result.payloadValid=true;
            result.payload={true,vertex.Point()}; result.evidence.hit=Evidence::Verified;
        }
        return result;
    }

    SCLineCurveIntersection3dWithStatus IntersectLineCurveWithStatus(
        const SCLine3d& line, const ISCCurve3d& curve, const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !line.IsValid(tolerance.distanceEpsilon) || !curve.IsValid(tolerance)) return {};
        if (const auto* segment=dynamic_cast<const SCLineCurve3d*>(&curve)) {
            SCVector3d first,second;
            if (Detail::TryUnitVector(line.direction,first) && Detail::TryUnitVector(segment->Line().direction,second)) {
                const auto cross=Cross(first,second);
                if (segment->ParameterRange().min < segment->ParameterRange().max &&
                    cross.x==0 && cross.y==0 && cross.z==0 && Distance(segment->PointAt(segment->ParameterRange().min),line,tolerance)<=tolerance.distanceEpsilon) {
                    SCLineCurveIntersection3dWithStatus result{};
                    result.state=State::NonUnique;
                    result.payload=Intersect(line,curve,tolerance);
                    result.evidence.hit=Evidence::Verified;
                    result.evidence.reasons=Reason::NonUnique;
                    return result;
                }
            }
        }
        const auto hit = Intersect(line, curve, tolerance);
        return LineCandidate<SCLineCurveIntersection3dWithStatus>(line, hit, hit.intersects ? curve.PointAt(hit.curveParameter) : SCPoint3d{}, tolerance, false);
    }

    SCLineCurveOnSurfaceIntersection3dWithStatus IntersectLineCurveOnSurfaceWithStatus(
        const SCLine3d& line, const SCCurveOnSurface& curve, const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !line.IsValid(tolerance.distanceEpsilon) || !curve.IsValid(tolerance)) return {};
        const auto hit = Intersect(line, curve, tolerance);
        return LineCandidate<SCLineCurveOnSurfaceIntersection3dWithStatus>(line, hit, hit.intersects ? curve.SupportSurface()->PointAt(hit.uv.x, hit.uv.y) : SCPoint3d{}, tolerance, false);
    }

    SCLineSurfaceIntersection3dWithStatus IntersectLineSurfaceWithStatus(
        const SCLine3d& line, const ISCSurface& surface, const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !line.IsValid(tolerance.distanceEpsilon) || !surface.IsValid(tolerance)) return {};
        if (const auto* plane = dynamic_cast<const SCPlaneSurface*>(&surface)) {
            const auto ur = plane->URange(), vr = plane->VRange();
            if (ur.min < ur.max && vr.min < vr.max) {
                const SCPolygon2d polygon(SCPolyline2d({{ur.min,vr.min},{ur.max,vr.min},{ur.max,vr.max},{ur.min,vr.max}}, SCPolylineClosure::Closed));
                const auto all = CollectPlanarRegionHits(line, *plane, polygon, tolerance);
                SCLineSurfaceIntersection3dWithStatus result{};
                Unknown(result, Reason::NoProof);
                if (all.continuous) {
                    result.state = State::NonUnique;
                    result.payload.liesOnSurface = true;
                    result.evidence.hit = Evidence::Verified;
                    result.evidence.negativeCompleteness = Evidence::NotApplicable;
                    result.evidence.reasons = Reason::NonUnique;
                    return result;
                }
                if (!all.hits.empty()) {
                    const auto& hit = all.hits.front();
                    result = LineCandidate<SCLineSurfaceIntersection3dWithStatus>(line, SCLineSurfaceIntersection3d{true, false, false,
                        hit.lineParameter, hit.u, hit.v, hit.point},
                        surface.PointAt(hit.u,hit.v), tolerance, false);
                }
                if (all.complete) {
                    result.evidence.negativeCompleteness = Evidence::Verified;
                    result.evidence.reasons = Reason::None;
                    if (all.hits.empty()) result.state = State::NoResult;
                }
                return result;
            }
        }
        const auto hit = Intersect(line, surface, tolerance);
        const auto actual = hit.intersects ? surface.PointAt(hit.u, hit.v) : SCPoint3d{};
        return LineCandidate<SCLineSurfaceIntersection3dWithStatus>(line, hit, actual, tolerance, false);
    }

    SCPlaneCurveIntersection3dWithStatus IntersectPlaneCurveWithStatus(
        const SCPlane& plane, const ISCCurve3d& curve, const SCGeometryTolerance3d& tolerance)
    {
        SCPlaneCurveIntersection3dWithStatus result{};
        if (!tolerance.IsValid() || !plane.IsValid(tolerance.distanceEpsilon) || !curve.IsValid(tolerance)) return result;
        Unknown(result, Reason::SearchIncomplete | Reason::NoProof);
        if (const auto* line = dynamic_cast<const SCLineCurve3d*>(&curve)) {
            const auto relation = IntersectLinePlaneWithStatus(line->Line(), plane, tolerance);
            if (relation.state==State::NonUnique && line->ParameterRange().min < line->ParameterRange().max) {
                result.state = State::NonUnique;
                result.evidence.hit = Evidence::Verified;
                result.evidence.negativeCompleteness = Evidence::NotApplicable;
                result.evidence.reasons = Reason::NonUnique;
                return result;
            }
            if (relation.evidence.negativeCompleteness==Evidence::Verified) {
                result.evidence.negativeCompleteness = Evidence::Verified;
                result.evidence.reasons = Reason::None;
                result.state = State::NoResult;
            }
        }
        const auto hit = Intersect(plane, curve, tolerance);
        if (hit.intersects && hit.IsValid() && Detail::SameRoundedPosition(curve.PointAt(hit.curveParameter), hit.point) &&
            Distance(hit.point, plane) <= tolerance.distanceEpsilon) {
            result.state = State::HasResult;
            result.payloadValid = true;
            result.payload = hit;
            result.evidence.hit = Evidence::Verified;
        }
        return result;
    }

    SCPlaneCurveOnSurfaceIntersection3dWithStatus IntersectPlaneCurveOnSurfaceWithStatus(
        const SCPlane& plane, const SCCurveOnSurface& curve, const SCGeometryTolerance3d& tolerance)
    {
        SCPlaneCurveOnSurfaceIntersection3dWithStatus result{};
        if (!tolerance.IsValid() || !plane.IsValid(tolerance.distanceEpsilon) || !curve.IsValid(tolerance)) return result;
        Unknown(result, Reason::SearchIncomplete | Reason::NoProof);
        const auto hit = Intersect(plane, curve, tolerance);
        if (hit.intersects && hit.IsValid() &&
            Detail::SameRoundedPosition(curve.SupportSurface()->PointAt(hit.uv.x, hit.uv.y), hit.point) &&
            Distance(hit.point, plane) <= tolerance.distanceEpsilon) {
            result.state = State::HasResult;
            result.payloadValid = true;
            result.payload = hit;
            result.evidence.hit = Evidence::Verified;
        }
        return result;
    }

    SCLineBrepFaceIntersection3dWithStatus IntersectLineBrepFaceWithStatus(
        const SCLine3d& line, const SCBrepFace& face, const SCGeometryTolerance3d& tolerance)
    {
        SCLineBrepFaceIntersection3dWithStatus result{};
        if (!tolerance.IsValid() || !line.IsValid(tolerance.distanceEpsilon) || !face.IsValid(tolerance)) return result;
        Unknown(result, Reason::RegionUnavailable);
        result.evidence.region = Evidence::Unknown;
        SCPolygon2d polygon;
        if (!Detail::TryBuildPolygonFromBrepFaceTrims(face, tolerance, polygon)) return result;
        result.evidence.region = Evidence::Verified;
        result.evidence.reasons = Reason::SearchIncomplete | Reason::NoProof;
        if (const auto* plane = dynamic_cast<const SCPlaneSurface*>(face.SupportSurface())) {
            const auto all = CollectPlanarRegionHits(line, *plane, polygon, tolerance);
            result.hits = all.hits;
            if (all.continuous) {
                result.state = all.hits.empty()?State::NonUnique:State::HasResult;
                result.evidence.hit = Evidence::Verified;
                result.evidence.negativeCompleteness = all.hits.empty()?Evidence::NotApplicable:Evidence::Unknown;
                result.evidence.reasons = Reason::NonUnique | (all.hits.empty()?Reason::None:Reason::NoProof);
                if (all.hits.empty()) return result;
            }
            if (all.complete && !all.continuous) {
                result.evidence.negativeCompleteness = Evidence::Verified;
                result.evidence.reasons = Reason::None;
                result.state = all.hits.empty() ? State::NoResult : State::HasResult;
            }
        } else {
            result.hits=Detail::CollectLineFaceHits(line,face,tolerance);
        }
        if (!result.hits.empty()) {
            result.state = State::HasResult;
            result.payloadValid = true;
            result.payload = result.hits.front();
            result.evidence.hit = Evidence::Verified;
            result.hitEvidence.assign(result.hits.size(), result.evidence);
        }
        return result;
    }

    SCLinePolyhedronFaceIntersection3dWithStatus IntersectLinePolyhedronFaceWithStatus(
        const SCLine3d& line, const PolyhedronFace3d& face, const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !line.IsValid(tolerance.distanceEpsilon) || !face.IsValid(tolerance)) return {};
        const auto projection = ProjectFaceToPolygon2d(face, tolerance);
        SCLinePolyhedronFaceIntersection3dWithStatus result{};
        Unknown(result, Reason::EvaluationUnavailable);
        if (!projection.success) return result;
        const SCPlaneSurface surface(face.SupportPlane(), projection.uAxis, projection.vAxis, {-1,1}, {-1,1});
        const auto all = CollectPlanarRegionHits(line, surface, projection.polygon, tolerance);
        result.evidence.region = Evidence::Verified;
        for (const auto& hit:all.hits)
            result.hits.push_back({true,hit.onBoundary,hit.lineParameter,hit.u,hit.v,hit.point});
        if (all.continuous) {
            result.state = all.hits.empty()?State::NonUnique:State::HasResult;
            result.evidence.hit = Evidence::Verified;
            result.evidence.negativeCompleteness = all.hits.empty()?Evidence::NotApplicable:Evidence::Unknown;
            result.evidence.reasons = Reason::NonUnique | (all.hits.empty()?Reason::None:Reason::NoProof);
        }
        if (!all.hits.empty()) {
            result.payload = result.hits.front();
            result.payloadValid = true;
            result.state = State::HasResult;
            result.evidence.hit = Evidence::Verified;
        } else if (all.complete && !all.continuous) result.state = State::NoResult;
        if (all.complete && !all.continuous) {
            result.evidence.negativeCompleteness = Evidence::Verified;
            result.evidence.reasons = Reason::None;
        }
        result.hitEvidence.assign(result.hits.size(),result.evidence);
        return result;
    }
    SCLineBrepBodyIntersection3dWithStatus IntersectLineBrepBodyWithStatus(
        const SCLine3d& line, const SCBrepBody& body, const SCGeometryTolerance3d& tolerance)
    {
        SCLineBrepBodyIntersection3dWithStatus result{};
        if (!tolerance.IsValid() || !line.IsValid(tolerance.distanceEpsilon) || !body.IsValid(tolerance)) return result;
        result.evidence.region = Evidence::Verified;
        result.evidence.negativeCompleteness = Evidence::Verified;
        result.evidence.reasons = Reason::None;
        bool continuous = false;
        std::size_t flat = 0;
        for (std::size_t shellIndex=0; shellIndex<body.ShellCount(); ++shellIndex) {
            const auto& shell = body.ShellAtRef(shellIndex);
            if (shell.FaceCount() > std::numeric_limits<std::size_t>::max()-flat) {
                Unknown(result, Reason::Unrepresentable);
                return result;
            }
            for (std::size_t local=0; local<shell.FaceCount(); ++local, ++flat) {
                SCBrepFace prepared;
                SCLineBrepFaceIntersection3dWithStatus faceResult{};
                if (Detail::TryPrepareFaceInBody(body, shell.FaceAtRef(local), tolerance, prepared))
                    faceResult = IntersectLineBrepFaceWithStatus(line, prepared, tolerance);
                else {
                    Unknown(faceResult, Reason::RegionUnavailable);
                    faceResult.evidence.region = Evidence::Unknown;
                }
                result.faces.push_back({shellIndex,local,flat,faceResult});
                result.evidence.reasons |= faceResult.evidence.reasons;
                if (faceResult.evidence.region != Evidence::Verified) result.evidence.region = Evidence::Unknown;
                continuous |= faceResult.state == State::NonUnique;
                if (faceResult.evidence.negativeCompleteness != Evidence::Verified)
                    result.evidence.negativeCompleteness = Evidence::Unknown;
                for (std::size_t i=0; i<faceResult.hits.size(); ++i) {
                    const auto& hit = faceResult.hits[i];
                    if (!hit.intersects || !hit.IsValid()) continue;
                    result.payload.faceIndices.push_back(flat);
                    result.payload.hits.push_back(hit);
                    result.hitEvidence.push_back(faceResult.hitEvidence.size() > i ? faceResult.hitEvidence[i] : faceResult.evidence);
                }
            }
        }
        if (!result.payload.hits.empty()) {
            result.payload.intersects = true;
            result.payloadValid = true;
            result.state = State::HasResult;
            result.evidence.hit = Evidence::Verified;
        } else if (continuous) {
            result.state = State::NonUnique;
            result.evidence.hit = Evidence::Verified;
            result.evidence.negativeCompleteness = Evidence::NotApplicable;
        } else if (result.evidence.negativeCompleteness == Evidence::Verified) result.state = State::NoResult;
        else result.state = State::Unknown;
        return result;
    }

    SCLinePolyhedronBodyIntersection3dWithStatus IntersectLinePolyhedronBodyWithStatus(
        const SCLine3d& line, const PolyhedronBody& body, const SCGeometryTolerance3d& tolerance)
    {
        SCLinePolyhedronBodyIntersection3dWithStatus result{};
        if (!tolerance.IsValid() || !line.IsValid(tolerance.distanceEpsilon) || !body.IsValid(tolerance)) return result;
        result.evidence.region = Evidence::Verified;
        result.evidence.negativeCompleteness = Evidence::Verified;
        result.evidence.reasons = Reason::None;
        bool continuous = false;
        for (std::size_t index=0; index<body.FaceCount(); ++index) {
            const auto face = IntersectLinePolyhedronFaceWithStatus(line, body.FaceAtRef(index), tolerance);
            result.faces.push_back(face);
            result.evidence.reasons |= face.evidence.reasons;
            continuous |= face.state == State::NonUnique;
            if (face.evidence.negativeCompleteness != Evidence::Verified)
                result.evidence.negativeCompleteness = Evidence::Unknown;
            for (std::size_t i=0;i<face.hits.size();++i) {
                result.payload.faceIndices.push_back(index);
                result.payload.hits.push_back(face.hits[i]);
                result.hitEvidence.push_back(face.hitEvidence[i]);
            }
        }
        if (!result.payload.hits.empty()) {
            result.payload.intersects = true;
            result.payloadValid = true;
            result.state = State::HasResult;
            result.evidence.hit = Evidence::Verified;
        } else if (continuous) {
            result.state = State::NonUnique;
            result.evidence.hit = Evidence::Verified;
            result.evidence.negativeCompleteness = Evidence::NotApplicable;
        } else result.state = result.evidence.negativeCompleteness == Evidence::Verified ? State::NoResult : State::Unknown;
        return result;
    }

    SCLineTriangleMeshIntersection3dWithStatus IntersectLineTriangleMeshWithStatus(
        const SCLine3d& line, const TriangleMesh& mesh, const SCGeometryTolerance3d& tolerance)
    {
        SCLineTriangleMeshIntersection3dWithStatus result{};
        if (!tolerance.IsValid() || !line.IsValid(tolerance.distanceEpsilon) || !mesh.IsValid(tolerance.distanceEpsilon)) return result;
        result.evidence.region = Evidence::Verified;
        result.evidence.negativeCompleteness = Evidence::Verified;
        result.evidence.reasons = Reason::None;
        bool continuous = false;
        for (std::size_t i=0; i<mesh.TriangleCount(); ++i) {
            const auto triangle = mesh.TriangleAt(i);
            SCVector3d ab,ac,normal;
            if (!Detail::TryUnitVector(triangle.b-triangle.a,ab) ||
                !Detail::TryUnitVector(triangle.c-triangle.a,ac) ||
                !Detail::TryUnitVector(Cross(ab,ac),normal)) {
                SCGeometryEvidenceStatus unavailable{};
                unavailable.region=Evidence::Unknown;
                unavailable.negativeCompleteness=Evidence::Unknown;
                unavailable.reasons=Reason::EvaluationUnavailable;
                result.triangles.push_back(unavailable);
                result.evidence.reasons|=unavailable.reasons;
                result.evidence.negativeCompleteness=Evidence::Unknown;
                continue;
            }
            const PolyhedronFace3d face(SCPlane::FromPointAndNormal(triangle.a, normal),
                PolyhedronLoop3d({triangle.a,triangle.b,triangle.c}));
            const auto hit = IntersectLinePolyhedronFaceWithStatus(line, face, tolerance);
            result.triangles.push_back(hit.evidence);
            result.evidence.reasons |= hit.evidence.reasons;
            continuous |= hit.state == State::NonUnique;
            if (hit.evidence.negativeCompleteness != Evidence::Verified)
                result.evidence.negativeCompleteness = Evidence::Unknown;
            if (!hit.payloadValid) continue;
            result.payload.triangleIndices.push_back(i);
            result.payload.lineParameters.push_back(hit.payload.lineParameter);
            result.payload.points.push_back(hit.payload.point);
            result.hitEvidence.push_back(hit.evidence);
        }
        if (!result.payload.points.empty()) {
            result.payload.intersects = true;
            result.payloadValid = true;
            result.state = State::HasResult;
            result.evidence.hit = Evidence::Verified;
        } else if (continuous) {
            result.state = State::NonUnique;
            result.evidence.hit = Evidence::Verified;
            result.evidence.negativeCompleteness = Evidence::NotApplicable;
        } else result.state = result.evidence.negativeCompleteness == Evidence::Verified ? State::NoResult : State::Unknown;
        return result;
    }

    namespace
    {
        SCPointContainment3dWithStatus Classification(SCPointContainment3d value, bool boundary)
        {
            SCPointContainment3dWithStatus result{};
            result.state = State::HasResult;
            result.payloadValid = true;
            result.payload = value;
            result.evidence.region = Evidence::Verified;
            result.evidence.hit = boundary ? Evidence::Verified : Evidence::NotApplicable;
            result.evidence.negativeCompleteness = boundary ? Evidence::NotApplicable : Evidence::Verified;
            result.evidence.reasons = Reason::None;
            return result;
        }

        template <typename Projection>
        SCPointContainment3dWithStatus ClassifyCandidate(const SCPoint3d& point, const Projection& projection,
            const SCGeometryTolerance3d& tolerance, bool face = false)
        {
            SCPointContainment3dWithStatus result{};
            if (projection.state == State::InvalidInput) return result;
            Unknown(result, projection.evidence.reasons | Reason::NoProof);
            result.evidence.region = projection.evidence.region;
            if (!projection.payloadValid) return result;
            if (Close(point, projection.payload.point, tolerance.distanceEpsilon))
                return Classification(face ? SCPointContainment3d::Inside : SCPointContainment3d::OnBoundary, !face);
            if (projection.evidence.optimality == Evidence::Verified)
                return Classification(SCPointContainment3d::Outside, false);
            return result;
        }

        struct ClosedTopology
        {
            struct Use { std::size_t face; bool forward; };
            std::map<std::pair<std::size_t,std::size_t>, std::vector<Use>> edges;
            std::map<std::size_t, std::set<std::size_t>> vertexFaces;
            std::set<std::vector<std::size_t>> faceKeys;
            bool valid{true};

            void AddLoop(const std::vector<std::size_t>& vertices, std::size_t face, bool outer)
            {
                if (vertices.size()<3) { valid=false; return; }
                std::set<std::size_t> unique(vertices.begin(),vertices.end());
                if (unique.size()!=vertices.size()) { valid=false; return; }
                if (outer && !faceKeys.emplace(unique.begin(),unique.end()).second) valid=false;
                for (std::size_t i=0; i<vertices.size(); ++i) {
                    const auto a=vertices[i], b=vertices[(i+1)%vertices.size()];
                    edges[{std::min(a,b),std::max(a,b)}].push_back({face,a<b});
                    vertexFaces[a].insert(face);
                }
            }

            bool Verify(std::size_t faceCount) const
            {
                if (!valid || faceCount<4 || edges.empty()) return false;
                std::vector<std::set<std::size_t>> neighbors(faceCount);
                std::map<std::size_t,std::map<std::size_t,std::set<std::size_t>>> links;
                for (const auto& [edge,uses] : edges) {
                    if (uses.size()!=2 || uses[0].forward==uses[1].forward || uses[0].face==uses[1].face) return false;
                    const auto a=uses[0].face, b=uses[1].face;
                    neighbors[a].insert(b); neighbors[b].insert(a);
                    for (auto vertex : {edge.first,edge.second}) {
                        links[vertex][a].insert(b); links[vertex][b].insert(a);
                    }
                }
                const auto connected = [](const auto& adjacency, std::size_t start, std::size_t count) {
                    std::set<std::size_t> seen{start};
                    std::vector<std::size_t> pending{start};
                    while (!pending.empty()) {
                        auto item=pending.back(); pending.pop_back();
                        for (auto next : adjacency.at(item)) if (seen.insert(next).second) pending.push_back(next);
                    }
                    return seen.size()==count;
                };
                if (!connected(neighbors,0,faceCount)) return false;
                for (const auto& [vertex,faces] : vertexFaces) {
                    const auto& link=links.at(vertex);
                    for (const auto& [face,adjacent] : link) if (adjacent.size()!=2) return false;
                    if (!connected(link,*faces.begin(),faces.size())) return false;
                }
                return true;
            }
        };

        bool ClosedMesh(const TriangleMesh& mesh)
        {
            ClosedTopology topology;
            for (std::size_t i=0; i<mesh.TriangleCount(); ++i) {
                const auto indices=mesh.TriangleIndicesAt(i);
                topology.AddLoop({indices[0],indices[1],indices[2]},i,true);
            }
            return topology.Verify(mesh.TriangleCount());
        }

        bool ClosedPolyhedron(const PolyhedronBody& body)
        {
            ClosedTopology topology;
            std::map<std::array<double,3>,std::size_t> vertices;
            const auto loopIds = [&](const PolyhedronLoop3d& loop) {
                std::vector<std::size_t> ids;
                for (const auto& p : loop.Vertices()) {
                    const auto [it,inserted]=vertices.emplace(std::array<double,3>{p.x,p.y,p.z},vertices.size());
                    ids.push_back(it->second);
                }
                return ids;
            };
            for (std::size_t i=0; i<body.FaceCount(); ++i) {
                const auto& face=body.FaceAtRef(i);
                topology.AddLoop(loopIds(face.OuterLoopRef()),i,true);
                for (std::size_t h=0; h<face.HoleCount(); ++h) topology.AddLoop(loopIds(face.HoleAtRef(h)),i,false);
            }
            return topology.Verify(body.FaceCount());
        }

        bool ClosedBrep(const SCBrepBody& body, const SCGeometryTolerance3d& tolerance)
        {
            if (body.ShellCount()!=1 || !body.ShellAtRef(0).IsClosed()) return false;
            const auto& shell=body.ShellAtRef(0);
            ClosedTopology topology;
            std::map<std::size_t,std::vector<ClosedTopology::Use>> edgeUses;
            for (std::size_t i=0; i<shell.FaceCount(); ++i) {
                const auto& face=shell.FaceAtRef(i);
                const auto* plane=dynamic_cast<const SCPlaneSurface*>(face.SupportSurface());
                if (plane==nullptr) return false;
                std::shared_ptr<ISCSurface> support(plane->Clone().release());
                const auto checkLoop = [&](const SCBrepLoop& loop, const SCCurveOnSurface* actual, bool outer) {
                    SCCurveOnSurface expected;
                    if (!Detail::TryBuildPlanarLoopTrim(body,loop,support,*plane,tolerance,expected)) return false;
                    if (actual!=nullptr && !Detail::ValidateTrimCorrespondence(
                        body,loop,*actual,support,*plane,tolerance)) return false;
                    std::vector<std::size_t> ids;
                    for (const auto& coedge : loop.Coedges()) {
                        if (coedge.EdgeIndex()>=body.EdgeCount()) return false;
                        edgeUses[coedge.EdgeIndex()].push_back({i,!coedge.Reversed()});
                        const auto& edge=body.Edges()[coedge.EdgeIndex()];
                        ids.push_back(coedge.Reversed()?edge.EndVertexIndex():edge.StartVertexIndex());
                    }
                    topology.AddLoop(ids,i,outer);
                    return true;
                };
                if (!checkLoop(face.OuterLoopRef(),face.OuterTrim().SupportSurface()?&face.OuterTrim():nullptr,true)) return false;
                for (std::size_t h=0; h<face.HoleCount(); ++h)
                    if (!checkLoop(face.HoleAt(h),face.HoleTrims().size()>h?&face.HoleTrims()[h]:nullptr,false)) return false;
            }
            for (const auto& [index,uses] : edgeUses)
                if (uses.size()!=2 || uses[0].forward==uses[1].forward || uses[0].face==uses[1].face) return false;
            return topology.Verify(shell.FaceCount());
        }

        // 浮点展开仅用于拓扑符号：TwoSum 和 FMA 保留每步的舍入余项。
        // 乘积接近下溢或任何中间量溢出时放弃证明，不猜测符号。
        struct RayExpansion { std::vector<double> terms; bool valid{true}; };

        RayExpansion RayScalar(double value)
        { return {{value},std::isfinite(value)}; }

        RayExpansion RayAdd(RayExpansion a,const RayExpansion& b)
        {
            if (!a.valid || !b.valid) return {{},false};
            for (const double value:b.terms) {
                double remainder=value;
                std::vector<double> next;
                for (const double term:a.terms) {
                    const double sum=remainder+term;
                    const double virtualTerm=sum-remainder;
                    const double error=(remainder-(sum-virtualTerm))+(term-virtualTerm);
                    if (!std::isfinite(sum) || !std::isfinite(virtualTerm) || !std::isfinite(error)) return {{},false};
                    if (error!=0) next.push_back(error);
                    remainder=sum;
                }
                if (remainder!=0 || next.empty()) next.push_back(remainder);
                a.terms=std::move(next);
            }
            return a;
        }

        RayExpansion RayNegate(RayExpansion value)
        { for (auto& term:value.terms) term=-term; return value; }

        RayExpansion RayMultiply(const RayExpansion& a,const RayExpansion& b)
        {
            if (!a.valid || !b.valid) return {{},false};
            RayExpansion result{{0},true};
            for (const double first:a.terms) for (const double second:b.terms) {
                if (first==0 || second==0) continue;
                const double product=first*second;
                // double 的精确乘积至多含 106 位；该界给余项留足表示空间。
                if (!std::isfinite(product) || std::abs(product)<std::ldexp(1.0,-900)) return {{},false};
                const double error=std::fma(first,second,-product);
                result=RayAdd(std::move(result),{{error,product},std::isfinite(error)});
                if (!result.valid) return result;
            }
            return result;
        }

        int RaySign(const RayExpansion& value)
        {
            for (auto it=value.terms.rbegin();it!=value.terms.rend();++it)
                if (*it!=0) return *it>0?1:-1;
            return 0;
        }

        // 将射线投影为原点，用两侧符号扰动 ±(δ,δ²) 约定共享边/顶点的
        // 半开归属。只在两个奇偶结果一致时发布；不按命中点距离去重。
        // 闭合流形及完整根集是调用前提，切触在相邻面贡献偶数次穿越。
        std::optional<bool> HalfOpenRing(const SCPoint3d& origin,
            const std::vector<SCPoint3d>& vertices, int perturbation)
        {
            const auto project = [&](const SCPoint3d& p) {
                const auto dx=RayAdd(RayScalar(p.x),RayScalar(-origin.x));
                const auto dy=RayAdd(RayScalar(p.y),RayScalar(-origin.y));
                const auto dz=RayAdd(RayScalar(p.z),RayScalar(-origin.z));
                return std::array<RayExpansion,2>{
                    RayAdd(dy,RayNegate(RayMultiply(dx,RayScalar(0.3713906763541037)))),
                    RayAdd(dz,RayNegate(RayMultiply(dx,RayScalar(0.52999894000318))))};
            };
            bool inside = false;
            for (std::size_t i=0; i<vertices.size(); ++i) {
                auto a=project(vertices[i]), b=project(vertices[(i+1)%vertices.size()]);
                if (!a[0].valid || !a[1].valid || !b[0].valid || !b[1].valid) return std::nullopt;
                // y=0 在符号查询点下方，因此顶点只由一条半开边拥有。
                const auto above=[&](const RayExpansion& y) { const int s=RaySign(y); return s>0 || (s==0 && perturbation<0); };
                if (above(a[1])==above(b[1])) continue;
                const auto det=RayAdd(RayMultiply(a[0],b[1]),RayNegate(RayMultiply(a[1],b[0])));
                if (!det.valid) return std::nullopt;
                int sign=RaySign(det);
                if (sign==0) {
                    // 精确常数项为零，依次比较符号扰动的 δ、δ² 系数。
                    const auto linear=RayAdd(a[1],RayNegate(b[1]));
                    const auto quadratic=RayAdd(b[0],RayNegate(a[0]));
                    if (!linear.valid || !quadratic.valid) return std::nullopt;
                    sign=perturbation*RaySign(linear);
                    if (sign==0) sign=perturbation*RaySign(quadratic);
                    if (sign==0) return std::nullopt;
                }
                const auto rise=RayAdd(b[1],RayNegate(a[1]));
                if (!rise.valid) return std::nullopt;
                if ((sign>0)==(RaySign(rise)>0)) inside=!inside;
            }
            return inside;
        }

        std::optional<bool> HalfOpenFace(const SCPoint3d& origin, const PolyhedronFace3d& face, int perturbation)
        {
            auto inside=HalfOpenRing(origin,face.OuterLoopRef().Vertices(),perturbation);
            if (!inside) return std::nullopt;
            for (std::size_t i=0; i<face.HoleCount(); ++i) {
                const auto hole=HalfOpenRing(origin,face.HoleAtRef(i).Vertices(),perturbation);
                if (!hole) return std::nullopt;
                if (*hole) *inside=false;
            }
            return inside;
        }

        std::optional<bool> HalfOpenBrepFace(const SCPoint3d& origin, const SCBrepBody& body,
            std::size_t faceIndex, int perturbation)
        {
            const auto& face=body.ShellAtRef(0).FaceAtRef(faceIndex);
            const auto ring=[&](const SCBrepLoop& loop) {
                std::vector<SCPoint3d> points;
                for (const auto& coedge:loop.Coedges()) {
                    const auto& edge=body.Edges()[coedge.EdgeIndex()];
                    points.push_back(body.Vertices()[coedge.Reversed()?edge.EndVertexIndex():edge.StartVertexIndex()].Point());
                }
                return HalfOpenRing(origin,points,perturbation);
            };
            auto inside=ring(face.OuterLoopRef());
            if (!inside) return std::nullopt;
            for (const auto& loop:face.HoleLoops()) {
                const auto hole=ring(loop);
                if (!hole) return std::nullopt;
                if (*hole) *inside=false;
            }
            return inside;
        }

        struct RayLoop
        {
            std::vector<std::size_t> vertices;
            std::vector<SCPoint3d> points;
            std::vector<std::array<std::size_t,2>> edges;
        };

        struct RayBoundaryRef
        {
            // 身份来自拓扑索引；不以命中坐标或浮点线参数作为合并键。
            std::array<std::size_t,3> identity;
            std::array<std::size_t,4> canonical;
            std::size_t incidentFaces{1};
        };

        using RayIncidence=std::map<std::array<std::size_t,3>,std::set<std::size_t>>;

        RayIncidence BuildRayIncidence(const std::vector<std::vector<RayLoop>>& faces)
        {
            RayIncidence result;
            for (std::size_t face=0;face<faces.size();++face)
                for (const auto& loop:faces[face])
                    for (std::size_t i=0;i<loop.vertices.size();++i) {
                        result[{0,loop.vertices[i],0}].insert(face);
                        result[{1,loop.edges[i][0],loop.edges[i][1]}].insert(face);
                    }
            return result;
        }

        std::optional<RayBoundaryRef> WithRayIncidence(std::optional<RayBoundaryRef> ref,const RayIncidence& incidence)
        {
            if (!ref || ref->identity[0]==2) return ref;
            const auto found=incidence.find(ref->identity);
            if (found==incidence.end()) return std::nullopt;
            ref->incidentFaces=found->second.size();
            return ref;
        }

        std::optional<RayBoundaryRef> ResolveRayBoundary(const SCPoint3d& point,
            const std::vector<RayLoop>& loops, std::size_t face, double epsilon)
        {
            std::optional<RayBoundaryRef> result;
            const auto accept=[&](RayBoundaryRef candidate) {
                if (result && result->identity!=candidate.identity) return false;
                if (!result || candidate.canonical<result->canonical) result=candidate;
                return true;
            };
            for (std::size_t l=0;l<loops.size();++l) {
                const auto& loop=loops[l];
                for (std::size_t i=0;i<loop.points.size();++i) {
                    if (!Close(point,loop.points[i],epsilon)) continue;
                    if (!Detail::SameRoundedPosition(point,loop.points[i])) return std::nullopt;
                    const auto coedge=std::min(i,(i+loop.points.size()-1)%loop.points.size());
                    if (!accept({{0,loop.vertices[i],0},{0,face,l,coedge}})) return std::nullopt;
                }
            }
            if (result) return result;
            for (std::size_t l=0;l<loops.size();++l) {
                const auto& loop=loops[l];
                for (std::size_t i=0;i<loop.points.size();++i) {
                    const auto& a=loop.points[i];
                    const auto& b=loop.points[(i+1)%loop.points.size()];
                    const auto edge=b-a;
                    const double length=std::hypot(edge.x,edge.y,edge.z);
                    if (!(length>0) || !std::isfinite(length)) return std::nullopt;
                    const double parameter=Dot(point-a,edge/length)/length;
                    if (!std::isfinite(parameter)) return std::nullopt;
                    if (parameter<0 || parameter>1) continue;
                    const SCPoint3d actual{std::lerp(a.x,b.x,parameter),std::lerp(a.y,b.y,parameter),
                        std::lerp(a.z,b.z,parameter)};
                    if (!Close(point,actual,epsilon)) continue;
                    if (!Detail::SameRoundedPosition(point,actual)) return std::nullopt;
                    if (!accept({{1,loop.edges[i][0],loop.edges[i][1]},{0,face,l,i}})) return std::nullopt;
                }
            }
            return result ? result : std::optional<RayBoundaryRef>{{{2,face,0},{0,face,0,0}}};
        }

        std::vector<RayLoop> BrepRayLoops(const SCBrepBody& body,std::size_t faceIndex)
        {
            const auto& face=body.ShellAtRef(0).FaceAtRef(faceIndex);
            std::vector<RayLoop> result;
            const auto append=[&](const SCBrepLoop& loop) {
                RayLoop ring;
                for (const auto& coedge:loop.Coedges()) {
                    const auto& edge=body.Edges()[coedge.EdgeIndex()];
                    const auto vertex=coedge.Reversed()?edge.EndVertexIndex():edge.StartVertexIndex();
                    ring.vertices.push_back(vertex);
                    ring.points.push_back(body.Vertices()[vertex].Point());
                    ring.edges.push_back({coedge.EdgeIndex(),0});
                }
                result.push_back(std::move(ring));
            };
            append(face.OuterLoopRef());
            for (const auto& hole:face.HoleLoops()) append(hole);
            return result;
        }

        std::vector<std::vector<RayLoop>> PolyhedronRayLoops(const PolyhedronBody& body)
        {
            std::map<std::array<double,3>,std::size_t> vertices;
            std::vector<std::vector<RayLoop>> result;
            for (const auto& face:body.Faces()) {
                std::vector<RayLoop> rings;
                const auto append=[&](const PolyhedronLoop3d& loop) {
                    RayLoop ring;
                    ring.points=loop.Vertices();
                    for (const auto& p:ring.points) {
                        const auto [it,inserted]=vertices.emplace(std::array<double,3>{p.x,p.y,p.z},vertices.size());
                        ring.vertices.push_back(it->second);
                    }
                    for (std::size_t i=0;i<ring.vertices.size();++i) {
                        const auto a=ring.vertices[i],b=ring.vertices[(i+1)%ring.vertices.size()];
                        ring.edges.push_back({std::min(a,b),std::max(a,b)});
                    }
                    rings.push_back(std::move(ring));
                };
                append(face.OuterLoopRef());
                for (std::size_t h=0;h<face.HoleCount();++h) append(face.HoleAtRef(h));
                result.push_back(std::move(rings));
            }
            return result;
        }

        template <typename Projection, typename Intersections, typename BoundaryOwner, typename BoundaryResolver>
        SCPointContainment3dWithStatus ClassifyBody(const SCPoint3d& point, const Projection& projection,
            const Intersections& intersections, bool closed, const SCGeometryTolerance3d& tolerance,
            const BoundaryOwner& ownsBoundary, const BoundaryResolver& resolveBoundary)
        {
            SCPointContainment3dWithStatus result{};
            Unknown(result, Reason::NoProof);
            result.evidence.region=projection.evidence.region;
            if (projection.payloadValid && Close(point,projection.payload.projection.point,tolerance.distanceEpsilon))
                return Classification(SCPointContainment3d::OnBoundary,true);
            // 完整求交可能发现局部投影遗漏的原点命中；开放体也保留该边界证据。
            for (const auto& hit : intersections.payload.hits) {
                if (hit.intersects && hit.IsValid() && Close(point,hit.point,tolerance.distanceEpsilon))
                    return Classification(SCPointContainment3d::OnBoundary,true);
            }
            if (!closed) {
                result.evidence.reasons |= Reason::NotClosed | Reason::TopologyInvalid;
                return result;
            }
            if (intersections.evidence.region!=Evidence::Verified ||
                intersections.evidence.negativeCompleteness!=Evidence::Verified) {
                result.evidence.reasons |= intersections.evidence.reasons;
                return result;
            }
            std::size_t crossings=0;
            struct BoundaryGroup { RayBoundaryRef owner; std::size_t representative; std::vector<std::size_t> hits; };
            std::map<std::array<std::size_t,3>,BoundaryGroup> groups;
            for (std::size_t i=0; i<intersections.payload.hits.size(); ++i) {
                const auto& hit=intersections.payload.hits[i];
                if (!hit.intersects || !hit.IsValid()) {
                    result.evidence.reasons |= Reason::TopologyInvalid;
                    return result;
                }
                // 不能把原点附近的参数直接丢弃并由剩余奇偶数定类。
                // 已核验的空间边界在上面处理；其余近零参数不能证明射线侧别。
                if (std::abs(hit.lineParameter)<=tolerance.parameterEpsilon) return result;
                if (!hit.onBoundary) {
                    if (hit.lineParameter>0) ++crossings;
                    continue;
                }
                const auto ref=resolveBoundary(intersections.payload.faceIndices[i],hit.point);
                if (!ref) return result;
                auto [it,inserted]=groups.try_emplace(ref->identity,BoundaryGroup{*ref,i,{}});
                if (ref->canonical<it->second.owner.canonical) {
                    it->second.owner=*ref;
                    it->second.representative=i;
                }
                it->second.hits.push_back(i);
            }
            for (const auto& [key,group]:groups) {
                std::array<std::size_t,2> local{};
                const auto& first=intersections.payload.hits[group.representative];
                std::set<std::size_t> faces;
                for (const auto i:group.hits) {
                    const auto face=intersections.payload.faceIndices[i];
                    const auto& hit=intersections.payload.hits[i];
                    if (!faces.insert(face).second || !Close(first.point,hit.point,tolerance.distanceEpsilon) ||
                        std::signbit(first.lineParameter)!=std::signbit(hit.lineParameter)) return result;
                    for (std::size_t side=0;side<2;++side) {
                        const auto owned=ownsBoundary(face,side==0?1:-1);
                        if (!owned) return result;
                        if (*owned) ++local[side];
                    }
                }
                if (faces.size()!=group.owner.incidentFaces) return result;
                // 每个拓扑事件单独验证相邻面的横穿奇偶；不同事件不能互相抵消歧义。
                if (local[0]%2!=local[1]%2) return result;
                if (local[0]%2 && first.lineParameter>0) ++crossings;
            }
            return Classification(crossings%2 ? SCPointContainment3d::Inside : SCPointContainment3d::Outside,false);
        }
    }

    SCPointPlaneSide3dWithStatus LocatePointWithStatus(const SCPoint3d& point, const SCPlane& plane,
        const SCGeometryTolerance3d& tolerance)
    {
        SCPointPlaneSide3dWithStatus result{};
        if (!tolerance.IsValid() || !point.IsValid() || !plane.IsValid(tolerance.distanceEpsilon)) return result;
        SCVector3d normal;
        if (!Detail::TryUnitVector(plane.normal,normal)) return result;
        const double distance=plane.SignedDistanceTo(point,tolerance.distanceEpsilon);
        if (!std::isfinite(distance)) { Unknown(result,Reason::Unrepresentable); return result; }
        result.state=State::HasResult;
        result.payloadValid=true;
        result.payload=distance>tolerance.distanceEpsilon ? SCPointPlaneSide3d::Above :
            distance < -tolerance.distanceEpsilon ? SCPointPlaneSide3d::Below : SCPointPlaneSide3d::OnPlane;
        result.evidence.hit=result.payload==SCPointPlaneSide3d::OnPlane ? Evidence::Verified : Evidence::NotApplicable;
        result.evidence.negativeCompleteness=result.payload==SCPointPlaneSide3d::OnPlane ? Evidence::NotApplicable : Evidence::Verified;
        result.evidence.reasons=Reason::None;
        return result;
    }

    SCPointContainment3dWithStatus LocatePointWithStatus(const SCPoint3d& point, const ISCCurve3d& target,
        const SCGeometryTolerance3d& tolerance)
    { return ClassifyCandidate(point,ProjectPointToCurveWithStatus(point,target,tolerance),tolerance); }

    SCPointContainment3dWithStatus LocatePointWithStatus(const SCPoint3d& point, const SCCurveOnSurface& target,
        const SCGeometryTolerance3d& tolerance)
    { return ClassifyCandidate(point,ProjectPointToCurveOnSurfaceWithStatus(point,target,tolerance),tolerance); }

    SCPointContainment3dWithStatus LocatePointWithStatus(const SCPoint3d& point, const SCBrepFace& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto projection=ProjectPointToBrepFaceWithStatus(point,target,tolerance);
        if (projection.payloadValid && Close(point,projection.payload.point,tolerance.distanceEpsilon))
            return Classification(projection.payload.onBoundary?SCPointContainment3d::OnBoundary:SCPointContainment3d::Inside,
                                  projection.payload.onBoundary);
        return ClassifyCandidate(point,projection,tolerance,true);
    }

    SCPointContainment3dWithStatus LocatePointWithStatus(const SCPoint3d& point, const PolyhedronFace3d& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto projection=ProjectPointToPolyhedronFaceWithStatus(point,target,tolerance);
        if (projection.payloadValid && Close(point,projection.payload.point,tolerance.distanceEpsilon))
            return Classification(projection.payload.onBoundary?SCPointContainment3d::OnBoundary:SCPointContainment3d::Inside,
                                  projection.payload.onBoundary);
        return ClassifyCandidate(point,projection,tolerance,true);
    }

    SCPointContainment3dWithStatus LocatePointWithStatus(const SCPoint3d& point, const SCBrepBody& body,
        const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !point.IsValid() || !body.IsValid(tolerance)) return {};
        const auto projection=ProjectPointToBrepBodyWithStatus(point,body,tolerance);
        SCPointContainment3dWithStatus result{};
        if (projection.payloadValid && Close(point,projection.payload.projection.point,tolerance.distanceEpsilon))
            result=Classification(SCPointContainment3d::OnBoundary,true);
        else {
            const auto line=SCLine3d::FromOriginAndDirection(point,{1.0,0.3713906763541037,0.52999894000318});
            const auto hits=IntersectLineBrepBodyWithStatus(line,body,tolerance);
            std::vector<std::vector<RayLoop>> loops;
            if (body.ShellCount()==1)
                for (std::size_t i=0;i<body.ShellAtRef(0).FaceCount();++i) loops.push_back(BrepRayLoops(body,i));
            const auto incidence=BuildRayIncidence(loops);
            result=ClassifyBody(point,projection,hits,ClosedBrep(body,tolerance),tolerance,
                [&](std::size_t index, int side) { return HalfOpenBrepFace(point,body,index,side); },
                [&](std::size_t index,const SCPoint3d& hit) {
                    return WithRayIncidence(ResolveRayBoundary(hit,loops[index],index,tolerance.distanceEpsilon),incidence);
                });
            for (const auto& face:hits.faces)
                result.faces.push_back({face.shellIndex,face.localFaceIndex,face.faceIndex,face.result.state,face.result.evidence});
        }
        if (result.faces.empty()) for (const auto& face:projection.faces)
            result.faces.push_back({face.shellIndex,face.localFaceIndex,face.faceIndex,face.result.state,face.result.evidence});
        return result;
    }

    SCPointContainment3dWithStatus LocatePointWithStatus(const SCPoint3d& point, const PolyhedronBody& body,
        const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !point.IsValid() || !body.IsValid(tolerance)) return {};
        const auto projection=ProjectPointToPolyhedronBodyWithStatus(point,body,tolerance);
        const auto line=SCLine3d::FromOriginAndDirection(point,{1.0,0.3713906763541037,0.52999894000318});
        const auto hits=IntersectLinePolyhedronBodyWithStatus(line,body,tolerance);
        const auto loops=PolyhedronRayLoops(body);
        const auto incidence=BuildRayIncidence(loops);
        auto result=ClassifyBody(point,projection,hits,ClosedPolyhedron(body),tolerance,
            [&](std::size_t index, int side) { return HalfOpenFace(point,body.FaceAtRef(index),side); },
            [&](std::size_t index,const SCPoint3d& hit) {
                return WithRayIncidence(ResolveRayBoundary(hit,loops[index],index,tolerance.distanceEpsilon),incidence);
            });
        for (std::size_t i=0;i<hits.faces.size();++i)
            result.faces.push_back({0,i,i,hits.faces[i].state,hits.faces[i].evidence});
        return result;
    }

    SCPointContainment3dWithStatus LocatePointWithStatus(const SCPoint3d& point, const TriangleMesh& mesh,
        const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !point.IsValid() || !mesh.IsValid(tolerance.distanceEpsilon)) return {};
        const auto projection=ProjectPointToTriangleMeshWithStatus(point,mesh,tolerance);
        if (projection.payloadValid && Close(point,projection.payload.point,tolerance.distanceEpsilon))
            return Classification(SCPointContainment3d::OnBoundary,true);
        const auto line=SCLine3d::FromOriginAndDirection(point,{1.0,0.3713906763541037,0.52999894000318});
        const auto hits=IntersectLineTriangleMeshWithStatus(line,mesh,tolerance);
        // 仅作共享分类器的数据视图，不构造另一个公共操作结果。
        struct MeshProjectionView {
            bool payloadValid;
            struct Payload { SCTriangleMeshProjection3d projection; } payload;
            SCGeometryEvidenceStatus evidence;
        };
        struct MeshHitsView {
            struct Payload {
                std::vector<SCLinePolyhedronFaceIntersection3d> hits;
                std::vector<std::size_t> faceIndices;
            } payload;
            SCGeometryEvidenceStatus evidence;
        };
        const MeshProjectionView projected{projection.payloadValid,{projection.payload},projection.evidence};
        MeshHitsView intersections{{},hits.evidence};
        for (std::size_t i=0; i<hits.payload.points.size(); ++i) {
            intersections.payload.faceIndices.push_back(hits.payload.triangleIndices[i]);
            intersections.payload.hits.push_back({true,true,hits.payload.lineParameters[i],0,0,hits.payload.points[i]});
        }
        std::vector<std::vector<RayLoop>> loops;
        for (std::size_t i=0;i<mesh.TriangleCount();++i) {
            const auto ids=mesh.TriangleIndicesAt(i);
            RayLoop loop;
            for (std::size_t j=0;j<3;++j) {
                loop.vertices.push_back(ids[j]);
                loop.points.push_back(mesh.VertexAt(ids[j]));
                loop.edges.push_back({std::min(ids[j],ids[(j+1)%3]),std::max(ids[j],ids[(j+1)%3])});
            }
            loops.push_back({std::move(loop)});
        }
        const auto incidence=BuildRayIncidence(loops);
        auto result=ClassifyBody(point,projected,intersections,ClosedMesh(mesh),tolerance,
            [&](std::size_t index,int side) {
                const auto tri=mesh.TriangleAt(index);
                return HalfOpenRing(point,{tri.a,tri.b,tri.c},side);
            },
            [&](std::size_t index,const SCPoint3d& hit) {
                return WithRayIncidence(ResolveRayBoundary(hit,loops[index],index,tolerance.distanceEpsilon),incidence);
            });
        for (std::size_t i=0;i<hits.triangles.size();++i) {
            const auto& evidence=hits.triangles[i];
            const bool hasHit=std::find(hits.payload.triangleIndices.begin(),hits.payload.triangleIndices.end(),i)!=hits.payload.triangleIndices.end();
            const State state=hasHit?State::HasResult:
                HasGeometryFailureReason(evidence.reasons,Reason::NonUnique)?State::NonUnique:
                evidence.negativeCompleteness==Evidence::Verified?State::NoResult:State::Unknown;
            result.faces.push_back({0,i,i,state,evidence});
        }
        return result;
    }

}
