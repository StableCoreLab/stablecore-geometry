#include "Core/Intersection.h"
#include "Support/StableNumerics.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <type_traits>
#include <utility>
#include <vector>

#include "Core/Projection.h"
#include "Core/Relation.h"
#include "Brep/PolyhedronBody.h"
#include "Brep/PolyhedronFace3d.h"
#include "Brep/SCBrepBody.h"
#include "Brep/SCBrepEdge.h"
#include "Brep/SCBrepFace.h"
#include "Brep/SCBrepVertex.h"
#include "../Detail/AffinePlaneGeometry.h"
#include "../Detail/BrepFaceRegion.h"
#include "../Detail/LineFaceHits.h"
#include "../Detail/CurveOnSurfaceNumerics.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Geometry3d/Detail/SurfaceNumerics.h"
#include "Geometry3d/SCOffsetSurface.h"
#include "Geometry3d/SCNurbsCurve3d.h"
#include "Geometry3d/SCNurbsSurface.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Geometry3d/TriangleMesh.h"
#include "Support/Epsilon.h"
#include "../Detail/ParametricSeeds.h"

namespace Geometry
{
    namespace
    {
        double Norm(const SCVector3d& vector) { return std::hypot(vector.x,vector.y,vector.z); }

        [[nodiscard]] double SurfaceResidualDistance(
            const SCLine3d& line, double t, const ISCSurface& surface, double u, double v)
        {
            return Norm(line.PointAt(t) - surface.PointAt(u, v));
        }

        [[nodiscard]] bool TryBuildSupportPlane(const SCNurbsSurface& surface,
                                                const SCGeometryTolerance3d& tolerance,
                                                SCPlane& plane)
        {
            if (!surface.IsValid(tolerance))
            {
                return false;
            }

            // This plane supplies a candidate only.  Floating-point PointAt
            // may depart from it even for an affine control net.
            if (surface.DegreeU() != 1 || surface.DegreeV() != 1 ||
                surface.ControlPointCountU() != 2 || surface.ControlPointCountV() != 2)
            {
                return false;
            }
            const auto& controlPoints = surface.ControlPoints();
            const auto& weights = surface.Weights();
            if (controlPoints.size() != 4 || weights.size() != 4 ||
                !std::all_of(weights.begin() + 1, weights.end(),
                             [&](double weight) { return weight == weights.front(); }))
            {
                return false;
            }
            const SCPoint3d affineCorner = controlPoints[1] + (controlPoints[2] - controlPoints[0]);
            if (controlPoints[3].x != affineCorner.x || controlPoints[3].y != affineCorner.y ||
                controlPoints[3].z != affineCorner.z)
            {
                return false;
            }

            const SCIntervald uRange = surface.URange();
            const SCIntervald vRange = surface.VRange();
            const double u = std::lerp(uRange.min, uRange.max, 0.5);
            const double v = std::lerp(vRange.min, vRange.max, 0.5);
            const SCSurfaceEval3d eval = surface.Evaluate(u, v, 1);
            const SCPoint3d surfacePoint = surface.PointAt(u, v);
            if (!Detail::HasIndependentSurfaceTangents(eval, tolerance.angleEpsilon) ||
                !surfacePoint.IsValid() || !Detail::SameRoundedPosition(eval.point, surfacePoint))
            {
                return false;
            }

            SCVector3d unitNormal;
            if (!Detail::TryUnitVector(eval.normal, unitNormal))
            {
                return false;
            }
            plane = SCPlane::FromPointAndNormal(surfacePoint, unitNormal);
            if (!plane.IsValid(tolerance.distanceEpsilon))
            {
                return false;
            }

            return true;
        }

        [[nodiscard]] bool TryProjectPointToAffinePlanarNurbs(const SCPoint3d& point,
                                                              const SCNurbsSurface& surface,
                                                              const SCGeometryTolerance3d& tolerance,
                                                              double& u,
                                                              double& v,
                                                              SCPoint3d& surfacePoint)
        {
            if (surface.DegreeU() != 1 || surface.DegreeV() != 1 || surface.ControlPointCountU() != 2 ||
                surface.ControlPointCountV() != 2)
            {
                return false;
            }

            const auto& controlPoints = surface.ControlPoints();
            if (controlPoints.size() != 4)
            {
                return false;
            }

            const SCPoint3d& p00 = controlPoints[0];
            const SCPoint3d& p10 = controlPoints[1];
            const SCPoint3d& p01 = controlPoints[2];
            const SCPoint3d& p11 = controlPoints[3];
            const auto& weights = surface.Weights();
            if (weights.size() != 4 ||
                !std::all_of(weights.begin() + 1, weights.end(),
                             [&](double weight) { return weight == weights.front(); }))
            {
                return false;
            }
            const SCPoint3d affineCorner = p10 + (p01 - p00);
            if (!affineCorner.AlmostEquals(p11, tolerance.distanceEpsilon))
            {
                return false;
            }

            const SCVector3d uDirection = p10 - p00;
            const SCVector3d vDirection = p01 - p00;
            const SCVector3d delta = point - p00;
            SCVector3d unitU, unitV;
            if (!Detail::TryUnitVector(uDirection, unitU) ||
                !Detail::TryUnitVector(vDirection, unitV))
            {
                return false;
            }
            const double uv = Dot(unitU, unitV);
            const double denominator = std::fma(-uv, uv, 1.0);
            if (!(denominator > tolerance.angleEpsilon * tolerance.angleEpsilon))
            {
                return false;
            }
            const double du = Dot(delta, unitU);
            const double dv = Dot(delta, unitV);
            const double normalizedU = (du - uv * dv) /
                (std::hypot(uDirection.x, uDirection.y, uDirection.z) * denominator);
            const double normalizedV = (dv - uv * du) /
                (std::hypot(vDirection.x, vDirection.y, vDirection.z) * denominator);
            if (!std::isfinite(normalizedU) || !std::isfinite(normalizedV))
            {
                return false;
            }
            if (normalizedU < -tolerance.parameterEpsilon || normalizedU > 1.0 + tolerance.parameterEpsilon ||
                normalizedV < -tolerance.parameterEpsilon || normalizedV > 1.0 + tolerance.parameterEpsilon)
            {
                return false;
            }

            const SCIntervald uRange = surface.URange();
            const SCIntervald vRange = surface.VRange();
            u = std::lerp(uRange.min, uRange.max, std::clamp(normalizedU, 0.0, 1.0));
            v = std::lerp(vRange.min, vRange.max, std::clamp(normalizedV, 0.0, 1.0));
            if (!std::isfinite(u) || !std::isfinite(v))
            {
                return false;
            }
            surfacePoint = surface.PointAt(u, v);
            return surfacePoint.AlmostEquals(point, tolerance.distanceEpsilon);
        }

        [[nodiscard]] SCLineSurfaceIntersection3d TryRefineLineSurfaceNewton(const SCLine3d& line,
                                                                             const ISCSurface& surface,
                                                                             double initialT,
                                                                             double initialU,
                                                                             double initialV,
                                                                             const SCGeometryTolerance3d& tolerance)
        {
            const SCIntervald uRange = surface.URange();
            const SCIntervald vRange = surface.VRange();
            double t = initialT;
            double u = std::clamp(initialU, uRange.min, uRange.max);
            double v = std::clamp(initialV, vRange.min, vRange.max);

            for (int iteration = 0; iteration < 32; ++iteration)
            {
                const SCPoint3d linePoint = line.PointAt(t);
                const SCSurfaceEval3d surfaceEval = surface.Evaluate(u, v, 1);
                if (dynamic_cast<const SCOffsetSurface*>(&surface) != nullptr ||
                    !Detail::HasIndependentSurfaceTangents(surfaceEval, tolerance.angleEpsilon))
                {
                    return {};
                }

                const SCPoint3d surfacePoint = surface.PointAt(u, v);
                if (!surfacePoint.IsValid() || !Detail::SameRoundedPosition(surfaceEval.point, surfacePoint))
                {
                    return {};
                }
                const SCVector3d residual = linePoint - surfacePoint;
                if (Norm(residual) <= tolerance.distanceEpsilon)
                {
                    return {true, false, false, t, u, v, surfacePoint};
                }

                const auto componentScale = [](const SCVector3d& value) {
                    return std::max({std::abs(value.x), std::abs(value.y), std::abs(value.z)});
                };
                const double scaleT = componentScale(line.direction);
                const double scaleU = componentScale(surfaceEval.derivativeU);
                const double scaleV = componentScale(surfaceEval.derivativeV);
                const double scaleR = componentScale(residual);
                if (!(scaleT > 0) || !(scaleU > 0) || !(scaleV > 0) ||
                    !(scaleR > 0) || !std::isfinite(scaleR)) return {};
                const SCVector3d columnT = line.direction / scaleT;
                const SCVector3d columnU = -surfaceEval.derivativeU / scaleU;
                const SCVector3d columnV = -surfaceEval.derivativeV / scaleV;
                const double determinant = Dot(columnT, Cross(columnU, columnV));
                const double determinantScale = Norm(columnT) * Norm(columnU) * Norm(columnV);
                if (!std::isfinite(determinant) ||
                    std::abs(determinant) <=
                        std::max(tolerance.angleEpsilon, std::numeric_limits<double>::epsilon()) *
                            determinantScale) return {};

                const SCVector3d rhs = -residual / scaleR;
                const double deltaT = (Dot(rhs, Cross(columnU, columnV)) / determinant * scaleR) / scaleT;
                const double deltaU = (Dot(columnT, Cross(rhs, columnV)) / determinant * scaleR) / scaleU;
                const double deltaV = (Dot(columnT, Cross(columnU, rhs)) / determinant * scaleR) / scaleV;
                if (!std::isfinite(deltaT) || !std::isfinite(deltaU) || !std::isfinite(deltaV))
                {
                    return {};
                }

                bool accepted = false;
                for (double scale = 1.0; scale >= 1.0 / 128.0; scale *= 0.5)
                {
                    const double candidateT = t + scale * deltaT;
                    const double candidateU = std::clamp(u + scale * deltaU, uRange.min, uRange.max);
                    const double candidateV = std::clamp(v + scale * deltaV, vRange.min, vRange.max);
                    const double candidateResidual = Norm(line.PointAt(candidateT) -
                        surface.PointAt(candidateU, candidateV));
                    if (candidateResidual < Norm(residual))
                    {
                        t = candidateT;
                        u = candidateU;
                        v = candidateV;
                        accepted = true;
                        break;
                    }
                }

                if (!accepted)
                {
                    return {};
                }
            }

            const SCPoint3d linePoint = line.PointAt(t);
            const SCPoint3d surfacePoint = surface.PointAt(u, v);
            if (linePoint.IsValid() && surfacePoint.IsValid() && Norm(linePoint-surfacePoint) <= tolerance.distanceEpsilon)
            {
                return {true, false, false, t, u, v, surfacePoint};
            }
            return {};
        }

        [[nodiscard]] SCLineCurveIntersection3d TryRefineLineCurveNewton(
            const SCLine3d& line, const ISCCurve3d& curve, double initialParameter,
            const SCGeometryTolerance3d& tolerance)
        {
            const auto range=curve.ParameterRange();
            SCVector3d unit;
            if (!range.IsValid() || !Detail::TryUnitVector(line.direction,unit)) return {};
            const auto removeLine=[&](const SCVector3d& v) { return v-unit*Dot(v,unit); };
            const auto residualAt=[&](double t) {
                const auto p=curve.PointAt(t);
                if (!p.IsValid()) return std::numeric_limits<double>::infinity();
                return Norm(removeLine(p-line.origin));
            };
            double parameter=std::clamp(initialParameter,range.min,range.max);
            double best=residualAt(parameter);
            if (!std::isfinite(best)) return {};
            for (int iteration=0;iteration<32 && best>tolerance.distanceEpsilon;++iteration) {
                const auto point=curve.PointAt(parameter);
                const auto eval=curve.Evaluate(parameter,2);
                if (!eval.IsValid() || eval.derivativeOrder<1 || !Detail::SameRoundedPosition(point,eval.point)) break;
                auto residual=removeLine(point-line.origin);
                auto first=removeLine(eval.firstDerivative);
                auto second=eval.derivativeOrder>=2?removeLine(eval.secondDerivative):SCVector3d{};
                const double scale=std::max({std::abs(residual.x),std::abs(residual.y),std::abs(residual.z),
                    std::abs(first.x),std::abs(first.y),std::abs(first.z),std::abs(second.x),std::abs(second.y),std::abs(second.z)});
                if (!(scale>0) || !std::isfinite(scale)) break;
                residual=residual/scale; first=first/scale; second=second/scale;
                const double tangent=Dot(first,first), gradient=Dot(residual,first);
                const double hessian=tangent+Dot(residual,second);
                const double denominator=hessian>std::numeric_limits<double>::epsilon()*tangent?hessian:tangent;
                if (!(denominator>0) || !std::isfinite(denominator)) break;
                const double delta=-gradient/denominator;
                if (!std::isfinite(delta)) break;
                bool accepted=false;
                for (double fraction=1;fraction>=1.0/128;fraction*=0.5) {
                    const double t=std::clamp(parameter+fraction*delta,range.min,range.max);
                    const double distance=residualAt(t);
                    if (distance<best) { parameter=t; best=distance; accepted=true; break; }
                }
                if (!accepted) break;
            }
            double step=std::max(range.max*0.25-range.min*0.25,tolerance.parameterEpsilon);
            for (int iteration=0;iteration<64 && best>tolerance.distanceEpsilon;++iteration) {
                bool improved=false;
                for (double delta : {-step,step}) {
                    const double t=std::clamp(parameter+delta,range.min,range.max);
                    const double distance=residualAt(t);
                    if (distance<best) { parameter=t; best=distance; improved=true; }
                }
                if (!improved) step*=0.5;
                if (step<=tolerance.parameterEpsilon) break;
            }
            const auto point=curve.PointAt(parameter);
            const auto projection=ProjectPointToLine(point,line,tolerance);
            if (!point.IsValid() || !projection.isOnLine || !std::isfinite(projection.parameter) ||
                !line.PointAt(projection.parameter).IsValid() || Norm(line.PointAt(projection.parameter)-point)>tolerance.distanceEpsilon) return {};
            return {true,projection.parameter,parameter,point};
        }

        [[nodiscard]] SCLineSurfaceIntersection3d RefineLineSurfaceIntersection(const SCLine3d& line,
                                                                              const ISCSurface& surface,
                                                                              double initialT,
                                                                              double initialU,
                                                                              double initialV,
                                                                              const SCGeometryTolerance3d& tolerance)
        {
            const SCIntervald uRange = surface.URange();
            const SCIntervald vRange = surface.VRange();
            double bestT = initialT;
            double bestU = std::clamp(initialU, uRange.min, uRange.max);
            double bestV = std::clamp(initialV, vRange.min, vRange.max);
            double bestResidualDistance = SurfaceResidualDistance(line, bestT, surface, bestU, bestV);

            double stepT = std::max(1.0, tolerance.parameterEpsilon * 8.0);
            double stepU = std::max((uRange.max*0.25-uRange.min*0.25), tolerance.parameterEpsilon);
            double stepV = std::max((vRange.max*0.25-vRange.min*0.25), tolerance.parameterEpsilon);

            for (int iteration = 0; iteration < 20; ++iteration)
            {
                bool improved = false;
                for (const double dt : std::array<double, 3>{-stepT, 0.0, stepT})
                {
                    for (const double du : std::array<double, 3>{-stepU, 0.0, stepU})
                    {
                        for (const double dv : std::array<double, 3>{-stepV, 0.0, stepV})
                        {
                            const double candidateT = bestT + dt;
                            const double candidateU = std::clamp(bestU + du, uRange.min, uRange.max);
                            const double candidateV = std::clamp(bestV + dv, vRange.min, vRange.max);
                            const double candidateResidualDistance =
                                SurfaceResidualDistance(line, candidateT, surface, candidateU, candidateV);
                            if (candidateResidualDistance <
                                bestResidualDistance)
                            {
                                bestT = candidateT;
                                bestU = candidateU;
                                bestV = candidateV;
                                bestResidualDistance = candidateResidualDistance;
                                improved = true;
                            }
                        }
                    }
                }

                if (!improved)
                {
                    stepT *= 0.5;
                    stepU *= 0.5;
                    stepV *= 0.5;
                }

                if (stepT <= tolerance.parameterEpsilon && stepU <= tolerance.parameterEpsilon &&
                    stepV <= tolerance.parameterEpsilon)
                {
                    break;
                }
            }

            const SCPoint3d pointOnLine = line.PointAt(bestT);
            const SCPoint3d pointOnSurface = surface.PointAt(bestU, bestV);
            if (!pointOnLine.IsValid() || !pointOnSurface.IsValid() || Norm(pointOnLine-pointOnSurface) > tolerance.distanceEpsilon)
            {
                return {};
            }

            return {true, false, false, bestT, bestU, bestV, pointOnSurface};
        }



    }  // namespace

    SCLinePlaneIntersection3d Intersect(const SCLine3d& line, const SCPlane& plane, const SCGeometryTolerance3d& tolerance)
    {
        SCLinePlaneIntersection3d result{};
        if (!tolerance.IsValid() || !line.IsValid(tolerance.distanceEpsilon) || !plane.IsValid(tolerance.distanceEpsilon)) return result;
        const auto unitNormal=plane.UnitNormal(tolerance.distanceEpsilon);
        const double scale=std::max({std::abs(line.direction.x),std::abs(line.direction.y),std::abs(line.direction.z)});
        const auto scaled=line.direction/scale;
        const double length=Norm(scaled);
        const double denominator=Dot(unitNormal,scaled/length);
        const double numerator=Dot(unitNormal,plane.origin-line.origin);
        if (!std::isfinite(denominator) || !std::isfinite(numerator)) return result;
        if (std::abs(denominator)<=tolerance.angleEpsilon) {
            result.isParallel=true;
            result.liesOnPlane=denominator==0.0 && std::abs(numerator)<=tolerance.distanceEpsilon;
            result.intersects=result.liesOnPlane;
            result.point=line.origin;
            return result;
        }
        const double parameter=((numerator/denominator)/length)/scale;
        const auto point=line.PointAt(parameter);
        if (!std::isfinite(parameter) || !point.IsValid() ||
            std::abs(plane.SignedDistanceTo(point,tolerance.distanceEpsilon))>tolerance.distanceEpsilon) return result;
        result.intersects=true; result.parameter=parameter; result.point=point;
        return result;
    }

    SCLineCurveIntersection3d Intersect(const SCLine3d& line, const ISCCurve3d& curve, const SCGeometryTolerance3d& tolerance)
    {
        if (!line.IsValid(tolerance.distanceEpsilon) || !curve.IsValid(tolerance))
        {
            return {};
        }

        if (const auto* lineCurve = dynamic_cast<const SCLineCurve3d*>(&curve))
        {
            const auto& curveLine = lineCurve->Line();
            SCVector3d unitA, unitB;
            if (!Detail::TryUnitVector(line.direction, unitA) ||
                !Detail::TryUnitVector(curveLine.direction, unitB)) return {};
            const auto cross = Cross(unitA, unitB);
            const double sine = Norm(cross);
            const auto range = lineCurve->ParameterRange();
            double parameter = range.min;
            if (sine > tolerance.angleEpsilon) {
                const auto delta = curveLine.origin - line.origin;
                const double scale = std::max({std::abs(curveLine.direction.x),
                    std::abs(curveLine.direction.y),std::abs(curveLine.direction.z)});
                const double length = Norm(curveLine.direction / scale);
                const double along = Dot(Cross(delta,unitA),cross/sine)/sine;
                parameter = (along/scale)/length;
                if (!Detail::StableContains(range.min,range.max,parameter,tolerance.parameterEpsilon)) return {};
                parameter = std::clamp(parameter,range.min,range.max);
            }
            const auto point = lineCurve->PointAt(parameter);
            const auto projection = ProjectPointToLine(point,line,tolerance);
            const auto onLine = line.PointAt(projection.parameter);
            if (!point.IsValid() || !onLine.IsValid() || !std::isfinite(projection.parameter) ||
                Norm(onLine-point)>tolerance.distanceEpsilon) return {};
            return {true,projection.parameter,parameter,point};
        }

        const SCIntervald range = curve.ParameterRange();
        if (!range.IsValid())
        {
            return {};
        }

        std::vector<double> curveBoundaries{range.min, range.max};
        if (const auto* nurbsCurve = dynamic_cast<const SCNurbsCurve3d*>(&curve))
        {
            auto nurbsBoundaries = Detail::KnotSpanBoundaries(
                range, nurbsCurve->Knots(), nurbsCurve->Degree(), nurbsCurve->ControlPoints().size());
            if (nurbsBoundaries.size() >= 2)
            {
                curveBoundaries = std::move(nurbsBoundaries);
            }
        }
        const SCBox3d curveBounds = curve.Bounds();
        const double curveScale = curveBounds.IsValid()
                                      ? (curveBounds.MaxPoint() - curveBounds.MinPoint()).Length()
                                      : tolerance.distanceEpsilon;
        const std::size_t spanCount = curveBoundaries.size() - 1;
        const std::size_t seedCount = std::max<std::size_t>(
            256, spanCount > std::numeric_limits<std::size_t>::max() / 3
                     ? std::numeric_limits<std::size_t>::max() : 3 * spanCount);
        const std::vector<double> curveSeeds = Detail::AdaptiveCurveParameterSeeds(
            curve, std::move(curveBoundaries), std::max(tolerance.distanceEpsilon * 4.0, curveScale * 1e-3),
            8, seedCount);

        double bestDistance = std::numeric_limits<double>::infinity();
        SCLineCurveIntersection3d best{};
        for (const double curveParameter : curveSeeds)
        {
            const SCPoint3d actual = curve.PointAt(curveParameter);
            if (!actual.IsValid()) continue;
            const auto projection = ProjectPointToLine(actual, line, tolerance);
            const auto onLine = line.PointAt(projection.parameter);
            const double distance = Norm(onLine - actual);
            if (onLine.IsValid() && std::isfinite(projection.parameter) && distance < bestDistance)
            {
                bestDistance = distance;
                best = {true, projection.parameter, curveParameter, actual};
            }
        }
        // Inspect every actual seed before refinement, so an approximate root
        // cannot hide an exactly represented knot or endpoint intersection.
        if (best.intersects && bestDistance <= tolerance.distanceEpsilon) return best;
        for (const double curveParameter : curveSeeds)
        {
            const auto refined = TryRefineLineCurveNewton(line, curve, curveParameter, tolerance);
            if (refined.intersects) return refined;
        }
        return {};
    }

    SCLineCurveOnSurfaceIntersection3d Intersect(const SCLine3d& line,
                                               const SCCurveOnSurface& curveOnSurface,
                                               const SCGeometryTolerance3d& tolerance)
    {
        if (!line.IsValid(tolerance.distanceEpsilon) || !curveOnSurface.IsValid(tolerance) ||
            curveOnSurface.PointCount() < 2)
        {
            return {};
        }

        const std::size_t pointCount = curveOnSurface.PointCount();
        const std::size_t segmentCount = curveOnSurface.UvCurve().IsClosed() ? pointCount : pointCount - 1;
        for (std::size_t i = 0; i < segmentCount; ++i)
        {
            const Detail::CurveOnSurfaceLineCandidate candidate =
                Detail::ProjectCurveOnSurfaceSegmentToLine(line, curveOnSurface, i);
            if (candidate.valid && candidate.distance <= tolerance.distanceEpsilon)
            {
                return {true, i, candidate.sample.parameter, candidate.lineParameter,
                        candidate.sample.uv, candidate.sample.point};
            }
        }

        return {};
    }

    SCLineSurfaceIntersection3d Intersect(const SCLine3d& line,
                                        const ISCSurface& surface,
                                        const SCGeometryTolerance3d& tolerance)
    {
        if (!line.IsValid(tolerance.distanceEpsilon) || !surface.IsValid(tolerance))
        {
            return {};
        }

        if (const auto* planeSurface = dynamic_cast<const SCPlaneSurface*>(&surface))
        {
            SCPlane actualPlane;
            if (!Detail::TryActualPlane(*planeSurface, tolerance, actualPlane))
            {
                return {};
            }
            const SCLinePlaneIntersection3d planeIntersection = Intersect(line, actualPlane, tolerance);
            if (!planeIntersection.intersects)
            {
                return {false, planeIntersection.isParallel, planeIntersection.liesOnPlane, 0.0, 0.0, 0.0, {}};
            }
            SCPoint2d uv;
            if (!Detail::TryAffinePlaneUv(*planeSurface, planeIntersection.point, tolerance, uv))
            {
                return {};
            }
            const auto ur=planeSurface->URange(),vr=planeSurface->VRange();
            const bool insideDomain = Detail::StableContains(ur.min,ur.max,uv.x,tolerance.parameterEpsilon) &&
                                      Detail::StableContains(vr.min,vr.max,uv.y,tolerance.parameterEpsilon);
            const SCPoint3d surfacePoint = planeSurface->PointAt(uv.x, uv.y);
            if (!insideDomain || !surfacePoint.IsValid() ||
                Norm(surfacePoint-planeIntersection.point) > tolerance.distanceEpsilon)
            {
                return {};
            }

            return {
                true, false, planeIntersection.liesOnPlane, planeIntersection.parameter, uv.x, uv.y, surfacePoint};
        }

        if (const auto* nurbsSurface = dynamic_cast<const SCNurbsSurface*>(&surface))
        {
            SCPlane supportPlane{};
            if (TryBuildSupportPlane(*nurbsSurface, tolerance, supportPlane))
            {
                const SCLinePlaneIntersection3d planeIntersection = Intersect(line, supportPlane, tolerance);
                if (planeIntersection.intersects && !planeIntersection.isParallel &&
                    std::isfinite(planeIntersection.parameter) && planeIntersection.point.IsValid())
                {
                    const auto onLine = [&](const SCPoint3d& candidate) {
                        if (!candidate.IsValid())
                        {
                            return false;
                        }
                        const SCPoint3d linePoint = line.PointAt(planeIntersection.parameter);
                        return linePoint.IsValid() &&
                            std::hypot(candidate.x - linePoint.x, candidate.y - linePoint.y,
                                       candidate.z - linePoint.z) <= tolerance.distanceEpsilon;
                    };
                    double u = 0.0;
                    double v = 0.0;
                    SCPoint3d surfacePoint{};
                    if (TryProjectPointToAffinePlanarNurbs(
                            planeIntersection.point, *nurbsSurface, tolerance, u, v, surfacePoint) &&
                        onLine(surfacePoint))
                    {
                        return {true, false, false, planeIntersection.parameter, u, v, surfacePoint};
                    }

                    const SCSurfaceProjection3d surfaceProjection =
                        ProjectPointToSurface(planeIntersection.point, surface, tolerance);
                    if (surfaceProjection.success && onLine(surfaceProjection.point) &&
                        surfaceProjection.point == surface.PointAt(surfaceProjection.u, surfaceProjection.v))
                    {
                        return {true, false, false, planeIntersection.parameter,
                                surfaceProjection.u, surfaceProjection.v, surfaceProjection.point};
                    }
                }
            }
        }

        const SCIntervald uRange = surface.URange();
        const SCIntervald vRange = surface.VRange();
        if (!uRange.IsValid() || !vRange.IsValid())
        {
            return {};
        }

        std::vector<double> uBoundaries{uRange.min, uRange.max};
        std::vector<double> vBoundaries{vRange.min, vRange.max};
        if (const auto* nurbsSurface = dynamic_cast<const SCNurbsSurface*>(&surface))
        {
            auto nurbsUBoundaries = Detail::KnotSpanBoundaries(
                uRange, nurbsSurface->KnotsU(), nurbsSurface->DegreeU(), nurbsSurface->ControlPointCountU());
            auto nurbsVBoundaries = Detail::KnotSpanBoundaries(
                vRange, nurbsSurface->KnotsV(), nurbsSurface->DegreeV(), nurbsSurface->ControlPointCountV());
            if (nurbsUBoundaries.size() >= 2 && nurbsVBoundaries.size() >= 2)
            {
                uBoundaries = std::move(nurbsUBoundaries);
                vBoundaries = std::move(nurbsVBoundaries);
            }
        }
        const SCBox3d surfaceBounds = surface.Bounds();
        const double surfaceScale = surfaceBounds.IsValid()
                                        ? (surfaceBounds.MaxPoint() - surfaceBounds.MinPoint()).Length()
                                        : tolerance.distanceEpsilon;
        const std::vector<Detail::SurfaceParameterSeed> surfaceSeeds = Detail::AdaptiveSurfaceParameterSeeds(
            surface, std::move(uBoundaries), std::move(vBoundaries),
            std::max(tolerance.distanceEpsilon * 4.0, surfaceScale * 1e-2));
        double bestU = std::lerp(uRange.min, uRange.max, 0.5);
        double bestV = std::lerp(vRange.min, vRange.max, 0.5);
        const SCPoint3d centerSurfacePoint = surface.PointAt(bestU, bestV);
        double bestT = ProjectPointToLine(centerSurfacePoint,line,tolerance).parameter;
        double bestResidualDistance = SurfaceResidualDistance(line, bestT, surface, bestU, bestV);
        const SCLineSurfaceIntersection3d centerNewton =
            TryRefineLineSurfaceNewton(line, surface, bestT, bestU, bestV, tolerance);
        if (centerNewton.intersects)
        {
            return centerNewton;
        }

        const SCSurfaceProjection3d originProjection = ProjectPointToSurface(line.origin, surface, tolerance);
        if (originProjection.success)
        {
            const double projectedLineParameter = ProjectPointToLine(originProjection.point,line,tolerance).parameter;
            const double projectedResidualDistance =
                SurfaceResidualDistance(line, projectedLineParameter, surface, originProjection.u, originProjection.v);
            if (projectedResidualDistance < bestResidualDistance)
            {
                bestT = projectedLineParameter;
                bestU = originProjection.u;
                bestV = originProjection.v;
                bestResidualDistance = projectedResidualDistance;
            }
            const SCLineSurfaceIntersection3d projectedNewton = TryRefineLineSurfaceNewton(
                line, surface, projectedLineParameter, originProjection.u, originProjection.v, tolerance);
            if (projectedNewton.intersects)
            {
                return projectedNewton;
            }
        }

        for (const Detail::SurfaceParameterSeed& seed : surfaceSeeds)
        {
            const double u = seed.u;
            const double v = seed.v;
            const SCPoint3d surfacePoint = surface.PointAt(u, v);
            const double t = ProjectPointToLine(surfacePoint,line,tolerance).parameter;
            const double residualDistance = SurfaceResidualDistance(line, t, surface, u, v);
            const SCLineSurfaceIntersection3d seedNewton =
                TryRefineLineSurfaceNewton(line, surface, t, u, v, tolerance);
            if (seedNewton.intersects)
            {
                return seedNewton;
            }
            if (residualDistance < bestResidualDistance)
            {
                bestT = t;
                bestU = u;
                bestV = v;
                bestResidualDistance = residualDistance;
            }
        }

        const SCLineSurfaceIntersection3d newtonResult =
            TryRefineLineSurfaceNewton(line, surface, bestT, bestU, bestV, tolerance);
        if (newtonResult.intersects)
        {
            return newtonResult;
        }
        return RefineLineSurfaceIntersection(line, surface, bestT, bestU, bestV, tolerance);
    }

    SCLineBrepEdgeIntersection3d Intersect(const SCLine3d& line, const SCBrepEdge& edge, const SCGeometryTolerance3d& tolerance)
    {
        if (!line.IsValid(tolerance.distanceEpsilon) || !edge.IsValid(tolerance) || edge.Curve() == nullptr)
        {
            return {};
        }

        const SCLineCurveIntersection3d curveIntersection = Intersect(line, *edge.Curve(), tolerance);
        return {curveIntersection.intersects,
                curveIntersection.lineParameter,
                curveIntersection.curveParameter,
                curveIntersection.point};
    }

    SCLineBrepVertexIntersection3d Intersect(const SCLine3d& line,
                                           const SCBrepVertex& vertex,
                                           const SCGeometryTolerance3d& tolerance)
    {
        if (!line.IsValid(tolerance.distanceEpsilon) || !vertex.IsValid())
        {
            return {};
        }

        const SCLineProjection3d projection = ProjectPointToLine(vertex.Point(), line, tolerance);
        const double residual = projection.point.IsValid()
            ? std::hypot(vertex.Point().x - projection.point.x,
                         vertex.Point().y - projection.point.y,
                         vertex.Point().z - projection.point.z)
            : std::numeric_limits<double>::infinity();
        if (!projection.isOnLine || !std::isfinite(residual) || residual > tolerance.distanceEpsilon)
        {
            return {};
        }

        return {true, projection.parameter, vertex.Point()};
    }

    SCLineBrepFaceIntersection3d Intersect(const SCLine3d& line, const SCBrepFace& face, const SCGeometryTolerance3d& tolerance)
    {
        if (!line.IsValid(tolerance.distanceEpsilon) || !face.IsValid(tolerance) || face.SupportSurface() == nullptr)
        {
            return {};
        }

        SCPolygon2d polygon;
        if (!Detail::TryBuildPolygonFromBrepFaceTrims(face, tolerance, polygon))
        {
            return {};
        }

        const SCLineSurfaceIntersection3d surfaceIntersection = Intersect(line, *face.SupportSurface(), tolerance);
        if (!surfaceIntersection.intersects || surfaceIntersection.liesOnSurface ||
            !std::isfinite(surfaceIntersection.lineParameter) ||
            !std::isfinite(surfaceIntersection.u) || !std::isfinite(surfaceIntersection.v) ||
            !surfaceIntersection.point.IsValid())
        {
            return {};
        }

        const SCPoint3d linePoint = line.PointAt(surfaceIntersection.lineParameter);
        const SCPoint3d surfacePoint = face.SupportSurface()->PointAt(surfaceIntersection.u, surfaceIntersection.v);
        const double residual = std::hypot(linePoint.x - surfacePoint.x,
                                           linePoint.y - surfacePoint.y,
                                           linePoint.z - surfacePoint.z);
        if (!linePoint.IsValid() || !surfacePoint.IsValid() || !std::isfinite(residual) ||
            residual > tolerance.distanceEpsilon)
        {
            return {};
        }

        const SCPointContainment2d containment = LocatePoint(
            SCPoint2d{surfaceIntersection.u, surfaceIntersection.v}, polygon, tolerance.parameterEpsilon);
        if (containment == SCPointContainment2d::Outside)
        {
            return {};
        }

        return {true,
                containment == SCPointContainment2d::OnBoundary,
                surfaceIntersection.lineParameter,
                surfaceIntersection.u,
                surfaceIntersection.v,
                surfacePoint};
    }

    std::vector<SCLineBrepFaceIntersection3d> Detail::CollectLineFaceHits(
        const SCLine3d& line, const SCBrepFace& face, const SCGeometryTolerance3d& tolerance)
    {
        std::vector<SCLineBrepFaceIntersection3d> hits;
        SCPolygon2d polygon;
        if (!tolerance.IsValid() || !line.IsValid(tolerance.distanceEpsilon) || !face.IsValid(tolerance) ||
            !Detail::TryBuildPolygonFromBrepFaceTrims(face,tolerance,polygon)) return hits;
        const auto& surface=*face.SupportSurface();
        const auto ur=surface.URange(), vr=surface.VRange();
        std::vector<double> ub{ur.min,ur.max},vb{vr.min,vr.max};
        if (const auto* nurbs=dynamic_cast<const SCNurbsSurface*>(&surface)) {
            ub=Detail::KnotSpanBoundaries(ur,nurbs->KnotsU(),nurbs->DegreeU(),nurbs->ControlPointCountU());
            vb=Detail::KnotSpanBoundaries(vr,nurbs->KnotsV(),nurbs->DegreeV(),nurbs->ControlPointCountV());
        }
        auto seeds=Detail::AdaptiveSurfaceParameterSeeds(surface,std::move(ub),std::move(vb),tolerance.distanceEpsilon*4);
        // 包含边界与不同盆地，不从某一个代表命中提前返回。
        for (int u=0;u<=8;++u) for (int v=0;v<=8;++v)
            seeds.push_back({std::lerp(ur.min,ur.max,u/8.0),std::lerp(vr.min,vr.max,v/8.0)});
        const auto append=[&](const SCLineSurfaceIntersection3d& hit) {
            if (!hit.intersects || hit.liesOnSurface || !hit.IsValid()) return;
            const auto actual=surface.PointAt(hit.u,hit.v);
            if (!Detail::SameRoundedPosition(hit.point,actual) ||
                !line.PointAt(hit.lineParameter).IsValid() || Norm(line.PointAt(hit.lineParameter)-actual)>tolerance.distanceEpsilon) return;
            const auto containment=LocatePoint(SCPoint2d{hit.u,hit.v},polygon,tolerance.parameterEpsilon);
            if (containment==SCPointContainment2d::Outside) return;
            for (const auto& previous:hits)
                if (std::abs(previous.lineParameter-hit.lineParameter)<=tolerance.parameterEpsilon &&
                    std::abs(previous.u-hit.u)<=tolerance.parameterEpsilon && std::abs(previous.v-hit.v)<=tolerance.parameterEpsilon) return;
            hits.push_back({true,containment==SCPointContainment2d::OnBoundary,hit.lineParameter,hit.u,hit.v,actual});
        };
        append(Intersect(line,surface,tolerance));
        for (const auto& seed:seeds) {
            const auto position=surface.PointAt(seed.u,seed.v);
            if (!position.IsValid()) continue;
            const auto projection=ProjectPointToLine(position,line,tolerance);
            if (!projection.isOnLine || !std::isfinite(projection.parameter)) continue;
            auto hit=TryRefineLineSurfaceNewton(line,surface,projection.parameter,seed.u,seed.v,tolerance);
            if (!hit.intersects) hit=RefineLineSurfaceIntersection(line,surface,projection.parameter,seed.u,seed.v,tolerance);
            append(hit);
        }
        std::sort(hits.begin(),hits.end(),[](const auto& a,const auto& b) {
            if (a.lineParameter!=b.lineParameter) return a.lineParameter<b.lineParameter;
            return std::tie(a.u,a.v)<std::tie(b.u,b.v);
        });
        return hits;
    }

    SCLineBrepBodyIntersection3d Intersect(const SCLine3d& line, const SCBrepBody& body, const SCGeometryTolerance3d& tolerance)
    {
        const auto result = IntersectLineBrepBodyWithStatus(line, body, tolerance);
        return result.payloadValid &&
            result.evidence.negativeCompleteness == SCGeometryEvidenceState::Verified
                ? result.payload : SCLineBrepBodyIntersection3d{};
    }

    SCLinePolyhedronFaceIntersection3d Intersect(const SCLine3d& line,
                                               const PolyhedronFace3d& face,
                                               const SCGeometryTolerance3d& tolerance)
    {
        if (!line.IsValid(tolerance.distanceEpsilon) || !face.IsValid(tolerance))
        {
            return {};
        }

        const SCLinePlaneIntersection3d planeIntersection = Intersect(line, face.SupportPlane(), tolerance);
        if (!planeIntersection.intersects)
        {
            return {};
        }

        const SCFaceProjection3d faceProjection = ProjectFaceToPolygon2d(face, tolerance);
        if (!faceProjection.success)
        {
            return {};
        }

        const SCVector3d delta = planeIntersection.point - faceProjection.origin;
        const double u = Dot(delta, faceProjection.uAxis);
        const double v = Dot(delta, faceProjection.vAxis);
        const SCPointContainment2d containment =
            LocatePoint(SCPoint2d{u, v}, faceProjection.polygon, tolerance.distanceEpsilon);
        if (containment == SCPointContainment2d::Outside)
        {
            return {};
        }

        return {true,
                containment == SCPointContainment2d::OnBoundary,
                planeIntersection.parameter,
                u,
                v,
                planeIntersection.point};
    }

    SCLinePolyhedronBodyIntersection3d Intersect(const SCLine3d& line,
                                               const PolyhedronBody& body,
                                               const SCGeometryTolerance3d& tolerance)
    {
        const auto result = IntersectLinePolyhedronBodyWithStatus(line, body, tolerance);
        return result.payloadValid &&
            result.evidence.negativeCompleteness == SCGeometryEvidenceState::Verified
                ? result.payload : SCLinePolyhedronBodyIntersection3d{};
    }

    SCLineTriangleMeshIntersection3d Intersect(const SCLine3d& line,
                                             const TriangleMesh& mesh,
                                             const SCGeometryTolerance3d& tolerance)
    {
        const auto result = IntersectLineTriangleMeshWithStatus(line, mesh, tolerance);
        return result.payloadValid &&
            result.evidence.negativeCompleteness == SCGeometryEvidenceState::Verified
                ? result.payload : SCLineTriangleMeshIntersection3d{};
    }

    SCPlaneCurveIntersection3d Intersect(const SCPlane& plane, const ISCCurve3d& curve, const SCGeometryTolerance3d& tolerance)
    {
        if (!plane.IsValid(tolerance.distanceEpsilon) || !curve.IsValid(tolerance))
        {
            return {};
        }

        if (const auto* lineCurve = dynamic_cast<const SCLineCurve3d*>(&curve))
        {
            const SCLinePlaneIntersection3d hit = Intersect(lineCurve->Line(), plane, tolerance);
            if (!hit.intersects)
            {
                return {};
            }
            const SCIntervald range = lineCurve->ParameterRange();
            if (!Detail::StableContains(range.min,range.max,hit.parameter,tolerance.parameterEpsilon))
            {
                return {};
            }
            return {true, hit.parameter, hit.point};
        }

        const SCIntervald range = curve.ParameterRange();
        if (!range.IsValid())
        {
            return {};
        }

        std::vector<double> parameterBoundaries{range.min, range.max};
        if (const auto* nurbsCurve = dynamic_cast<const SCNurbsCurve3d*>(&curve))
        {
            auto nurbsBoundaries = Detail::KnotSpanBoundaries(
                range, nurbsCurve->Knots(), nurbsCurve->Degree(), nurbsCurve->ControlPoints().size());
            if (nurbsBoundaries.size() >= 2)
            {
                parameterBoundaries = std::move(nurbsBoundaries);
            }
        }
        const SCBox3d curveBounds = curve.Bounds();
        const double curveScale = curveBounds.IsValid()
                                      ? (curveBounds.MaxPoint() - curveBounds.MinPoint()).Length()
                                      : tolerance.distanceEpsilon;
        const bool customCurve = dynamic_cast<const SCNurbsCurve3d*>(&curve) == nullptr;
        const std::size_t spanCount = parameterBoundaries.size() - 1;
        const std::size_t seedCount = customCurve ? 256 :
            std::max<std::size_t>(256, spanCount > std::numeric_limits<std::size_t>::max() / 3
                ? std::numeric_limits<std::size_t>::max() : 3 * spanCount);
        std::vector<double> parameterSeeds = Detail::AdaptiveCurveParameterSeeds(
            curve, std::move(parameterBoundaries), std::max(tolerance.distanceEpsilon * 4.0, curveScale * 1e-3),
            8, customCurve ? seedCount - 33 : seedCount);
        if (customCurve)
        {
            for (int sample = 0; sample <= 32; ++sample)
            {
                parameterSeeds.push_back(std::lerp(range.min, range.max, sample / 32.0));
            }
            std::sort(parameterSeeds.begin(), parameterSeeds.end());
            parameterSeeds.erase(std::unique(parameterSeeds.begin(), parameterSeeds.end()), parameterSeeds.end());
        }
        const SCVector3d unitNormal = plane.UnitNormal(tolerance.distanceEpsilon);

        const auto refineTangentRoot = [&](double initialParameter) -> std::optional<double> {
            double parameter = std::clamp(initialParameter, range.min, range.max);
            for (int iteration = 0; iteration < 32; ++iteration)
            {
                const SCCurveEval3d eval = curve.Evaluate(parameter, 1);
                const SCPoint3d curvePoint = curve.PointAt(parameter);
                if (!eval.IsValid() || eval.derivativeOrder < 1 || !curvePoint.IsValid() ||
                    !Detail::SameRoundedPosition(eval.point, curvePoint))
                {
                    return std::nullopt;
                }
                const double distance = plane.SignedDistanceTo(curvePoint, tolerance.distanceEpsilon);
                if (std::abs(distance) <= tolerance.distanceEpsilon)
                {
                    return parameter;
                }
                const double derivative = Dot(unitNormal, eval.firstDerivative);
                const double derivativeScale = std::hypot(eval.firstDerivative.x,eval.firstDerivative.y,eval.firstDerivative.z);
                if (!std::isfinite(derivative) || !std::isfinite(derivativeScale) ||
                    std::abs(derivative) <= std::numeric_limits<double>::epsilon() * derivativeScale)
                {
                    return std::nullopt;
                }
                bool accepted = false;
                for (double scale = 1.0; scale >= 1.0 / 128.0; scale *= 0.5)
                {
                    const auto advanced=Detail::StableAdvance(parameter,-scale*(distance/derivative));
                    if (!advanced) continue;
                    const double candidate = std::clamp(*advanced, range.min, range.max);
                    const double candidateDistance =
                        plane.SignedDistanceTo(curve.PointAt(candidate), tolerance.distanceEpsilon);
                    if (std::isfinite(candidateDistance) && std::abs(candidateDistance) < std::abs(distance))
                    {
                        parameter = candidate;
                        accepted = true;
                        break;
                    }
                }
                if (!accepted)
                {
                    return std::nullopt;
                }
            }

            return std::nullopt;
        };

        for (std::size_t i = 0; i < parameterSeeds.size(); ++i)
        {
            const double t0 = parameterSeeds[i];
            const SCPoint3d p0 = curve.PointAt(t0);
            if (!p0.IsValid())
            {
                continue;
            }
            const double d0 = plane.SignedDistanceTo(p0, tolerance.distanceEpsilon);
            if (std::isfinite(d0) && std::abs(d0) <= tolerance.distanceEpsilon)
            {
                return {true, t0, p0};
            }

            if (i + 1 < parameterSeeds.size())
            {
                double low = t0;
                double high = parameterSeeds[i + 1];
                const double d1 = plane.SignedDistanceTo(curve.PointAt(high), tolerance.distanceEpsilon);
                if (std::isfinite(d0) && std::isfinite(d1) &&
                    ((d0 < 0.0 && d1 > 0.0) || (d0 > 0.0 && d1 < 0.0)))
                {
                    for (int iteration = 0; iteration < 64; ++iteration)
                    {
                        const double middle = std::lerp(low, high, 0.5);
                        if (middle <= low || middle >= high)
                        {
                            break;
                        }
                        const SCPoint3d middlePoint = curve.PointAt(middle);
                        if (!middlePoint.IsValid())
                        {
                            break;
                        }
                        const double middleDistance = plane.SignedDistanceTo(middlePoint, tolerance.distanceEpsilon);
                        if (std::isfinite(middleDistance) && std::abs(middleDistance) <= tolerance.distanceEpsilon)
                        {
                            return {true, middle, middlePoint};
                        }
                        if ((d0 < 0.0) == (middleDistance < 0.0))
                        {
                            low = middle;
                        }
                        else
                        {
                            high = middle;
                        }
                    }
                    const double parameter = std::lerp(low, high, 0.5);
                    const SCPoint3d point = curve.PointAt(parameter);
                    if (point.IsValid() && std::abs(plane.SignedDistanceTo(point, tolerance.distanceEpsilon)) <=
                        tolerance.distanceEpsilon)
                    {
                        return {true, parameter, point};
                    }
                    continue;
                }
            }

            if (const std::optional<double> tangentRoot = refineTangentRoot(t0))
            {
                const SCPoint3d point = curve.PointAt(*tangentRoot);
                if (point.IsValid() &&
                    std::abs(plane.SignedDistanceTo(point, tolerance.distanceEpsilon)) <= tolerance.distanceEpsilon)
                {
                    return {true, *tangentRoot, point};
                }
            }
        }

        return {};
    }

    SCPlaneCurveOnSurfaceIntersection3d Intersect(const SCPlane& plane,
                                                const SCCurveOnSurface& curveOnSurface,
                                                const SCGeometryTolerance3d& tolerance)
    {
        if (!plane.IsValid(tolerance.distanceEpsilon) || !curveOnSurface.IsValid(tolerance) ||
            curveOnSurface.PointCount() < 2)
        {
            return {};
        }

        const std::size_t pointCount = curveOnSurface.PointCount();
        const std::size_t segmentCount = curveOnSurface.UvCurve().IsClosed() ? pointCount : pointCount - 1;
        for (std::size_t i = 0; i < segmentCount; ++i)
        {
            Detail::CurveOnSurfaceSample previous{};
            double previousDistance = 0.0;
            constexpr int sampleCount = 32;
            for (int sampleIndex = 0; sampleIndex <= sampleCount; ++sampleIndex)
            {
                const double parameter = static_cast<double>(sampleIndex) / sampleCount;
                const Detail::CurveOnSurfaceSample current =
                    Detail::EvaluateCurveOnSurfaceSegment(curveOnSurface, i, parameter);
                if (!current.valid)
                {
                    previous = {};
                    continue;
                }
                const double distance = plane.SignedDistanceTo(current.point, tolerance.distanceEpsilon);
                if (!std::isfinite(distance))
                {
                    previous = {};
                    continue;
                }
                if (std::abs(distance) <= tolerance.distanceEpsilon)
                {
                    return {true, i, current.parameter, current.uv, current.point};
                }
                if (previous.valid &&
                    ((previousDistance < 0.0 && distance > 0.0) ||
                     (previousDistance > 0.0 && distance < 0.0)))
                {
                    double low = previous.parameter;
                    double high = current.parameter;
                    double lowDistance = previousDistance;
                    for (int iteration = 0; iteration < 64; ++iteration)
                    {
                        const double middle = std::lerp(low, high, 0.5);
                        if (middle <= low || middle >= high)
                        {
                            break;
                        }
                        const Detail::CurveOnSurfaceSample midpoint =
                            Detail::EvaluateCurveOnSurfaceSegment(curveOnSurface, i, middle);
                        if (!midpoint.valid)
                        {
                            break;
                        }
                        const double middleDistance =
                            plane.SignedDistanceTo(midpoint.point, tolerance.distanceEpsilon);
                        if (!std::isfinite(middleDistance))
                        {
                            break;
                        }
                        if (std::abs(middleDistance) <= tolerance.distanceEpsilon)
                        {
                            return {true, i, midpoint.parameter, midpoint.uv, midpoint.point};
                        }
                        if ((lowDistance < 0.0 && middleDistance < 0.0) ||
                            (lowDistance > 0.0 && middleDistance > 0.0))
                        {
                            low = middle;
                            lowDistance = middleDistance;
                        }
                        else
                        {
                            high = middle;
                        }
                    }
                }
                previous = current;
                previousDistance = distance;
            }
        }

        return {};
    }

    SCPlaneBrepEdgeIntersection3d Intersect(const SCPlane& plane,
                                          const SCBrepEdge& edge,
                                          const SCGeometryTolerance3d& tolerance)
    {
        if (!plane.IsValid(tolerance.distanceEpsilon) || !edge.IsValid(tolerance) || edge.Curve() == nullptr)
        {
            return {};
        }

        const SCPlaneCurveIntersection3d hit = Intersect(plane, *edge.Curve(), tolerance);
        if (!hit.intersects)
        {
            return {};
        }
        return {true, hit.curveParameter, hit.point};
    }

    SCPlaneBrepVertexIntersection3d Intersect(const SCPlane& plane,
                                            const SCBrepVertex& vertex,
                                            const SCGeometryTolerance3d& tolerance)
    {
        if (!plane.IsValid(tolerance.distanceEpsilon) || !vertex.IsValid())
        {
            return {};
        }

        if (std::abs(plane.SignedDistanceTo(vertex.Point(), tolerance.distanceEpsilon)) > tolerance.distanceEpsilon)
        {
            return {};
        }

        return {true, vertex.Point()};
    }

    SCPlanePlaneIntersection3d Intersect(const SCPlane& first, const SCPlane& second, const SCGeometryTolerance3d& tolerance)
    {
        const auto result=IntersectPlanePlaneWithStatus(first,second,tolerance);
        return result.payloadValid || result.state==SCGeometryOperationState::NonUnique
            ? result.payload : SCPlanePlaneIntersection3d{};
    }
}  // namespace Geometry

