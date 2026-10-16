#include "Core/Projection.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "Core/Relation.h"
#include "Core/Triangulation.h"
#include "../Detail/AffinePlaneGeometry.h"
#include "../Detail/BrepFaceRegion.h"
#include "../Detail/CurveOnSurfaceNumerics.h"
#include "../Detail/ParametricSeeds.h"
#include "../Detail/SurfaceProjectionSolver.h"
#include "../Detail/ProjectionCandidates.h"
#include "../Detail/PolylineRing2d.h"
#include "Brep/PolyhedronBody.h"
#include "Brep/PolyhedronFace3d.h"
#include "Brep/SCBrepBody.h"
#include "Brep/SCBrepEdge.h"
#include "Brep/SCBrepFace.h"
#include "Brep/SCBrepVertex.h"
#include "Geometry3d/SCLineCurve3d.h"
#include "Geometry3d/Detail/SurfaceNumerics.h"
#include "Geometry3d/SCNurbsCurve3d.h"
#include "Geometry3d/SCNurbsSurface.h"
#include "Geometry3d/SCOffsetSurface.h"
#include "Geometry3d/SCPlaneSurface.h"
#include "Geometry3d/TriangleMesh.h"
#include "Support/Epsilon.h"

namespace Geometry
{
    namespace
    {
        double CandidateDistance(const SCPoint3d& query, const SCPoint3d& candidate)
        { return std::hypot(query.x-candidate.x,query.y-candidate.y,query.z-candidate.z); }

        struct PlaneProjectionBasis
        {
            SCVector3d u{};
            SCVector3d v{};
        };

        PlaneProjectionBasis BuildPlaneProjectionBasis(const SCPlane& plane, double eps)
        {
            const SCVector3d normal = plane.UnitNormal(eps);
            const SCVector3d axis =
                std::abs(normal.x) <= std::abs(normal.y) && std::abs(normal.x) <= std::abs(normal.z)
                    ? SCVector3d{1.0, 0.0, 0.0}
                    : (std::abs(normal.y) <= std::abs(normal.z) ? SCVector3d{0.0, 1.0, 0.0} : SCVector3d{0.0, 0.0, 1.0});
            const SCVector3d u = Cross(normal, axis).Normalized(eps);
            const SCVector3d v = Cross(normal, u).Normalized(eps);
            return {u, v};
        }

        SCPoint2d ProjectToLocalPlaneCoordinates(const SCPoint3d& point,
                                               const SCPlane& plane,
                                               const PlaneProjectionBasis& basis)
        {
            const SCVector3d delta = point - plane.origin;
            return SCPoint2d{Dot(delta, basis.u), Dot(delta, basis.v)};
        }

        double SignedArea2d(const std::vector<SCPoint2d>& points)
        {
            if (points.size() < 3)
            {
                return 0.0;
            }

            double area = 0.0;
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                const SCPoint2d& current = points[i];
                const SCPoint2d& next = points[(i + 1) % points.size()];
                area += current.x * next.y - next.x * current.y;
            }

            return 0.5 * area;
        }

        void EnsureRingOrientation(std::vector<SCPoint2d>& points, bool counterClockwise)
        {
            if (points.size() < 3)
            {
                return;
            }

            const double signedArea = SignedArea2d(points);
            if ((counterClockwise && signedArea < 0.0) || (!counterClockwise && signedArea > 0.0))
            {
                std::reverse(points.begin(), points.end());
            }
        }

        struct HomogeneousCurvePoint
        {
            double x{0.0};
            double y{0.0};
            double z{0.0};
            double weight{0.0};
        };

        HomogeneousCurvePoint Interpolate(const HomogeneousCurvePoint& first,
                                          const HomogeneousCurvePoint& second,
                                          double parameter)
        {
            return {first.x + (second.x - first.x) * parameter,
                    first.y + (second.y - first.y) * parameter,
                    first.z + (second.z - first.z) * parameter,
                    first.weight + (second.weight - first.weight) * parameter};
        }

        bool BezierControlsForNurbsSpan(const SCNurbsCurve3d& curve,
                                        std::size_t span,
                                        std::vector<HomogeneousCurvePoint>& controls,
                                        double& coordinateScale)
        {
            const std::size_t degree = static_cast<std::size_t>(curve.Degree());
            const auto& knots = curve.Knots();
            const auto& points = curve.ControlPoints();
            const auto& weights = curve.Weights();
            double weightScale = 0.0;
            coordinateScale = 1.0;
            for (std::size_t j = 0; j <= degree; ++j)
            {
                const std::size_t index = span - degree + j;
                weightScale = std::max(weightScale, weights[index]);
                coordinateScale = std::max({coordinateScale, std::abs(points[index].x),
                                            std::abs(points[index].y), std::abs(points[index].z)});
            }

            std::vector<HomogeneousCurvePoint> active(degree + 1);
            for (std::size_t j = 0; j <= degree; ++j)
            {
                const std::size_t index = span - degree + j;
                const double weight = weights[index] / weightScale;
                if (weight < 64.0 * std::numeric_limits<double>::epsilon())
                {
                    return false;
                }
                active[j] = {points[index].x / coordinateScale * weight,
                             points[index].y / coordinateScale * weight,
                             points[index].z / coordinateScale * weight, weight};
            }

            controls.resize(degree + 1);
            for (std::size_t control = 0; control <= degree; ++control)
            {
                auto work = active;
                for (std::size_t level = 1; level <= degree; ++level)
                {
                    const double argument = level <= degree - control ? knots[span] : knots[span + 1];
                    for (std::size_t j = degree; j >= level; --j)
                    {
                        const std::size_t index = span - degree + j;
                        const double denominator = knots[index + degree - level + 1] - knots[index];
                        if (!(denominator > 0.0) || !std::isfinite(denominator))
                        {
                            return false;
                        }
                        const double fraction = (argument - knots[index]) / denominator;
                        if (!std::isfinite(fraction) || fraction < 0.0 || fraction > 1.0)
                        {
                            return false;
                        }
                        work[j] = Interpolate(work[j - 1], work[j], fraction);
                    }
                }
                controls[control] = work[degree];
            }
            return true;
        }

        bool BoundHomogeneousControls(const std::vector<HomogeneousCurvePoint>& controls,
                                     double coordinateScale,
                                     SCBox3d& bounds)
        {
            for (const auto& control : controls)
            {
                if (!(control.weight > 0.0) || !std::isfinite(control.weight))
                {
                    return false;
                }
                const SCPoint3d point{control.x / control.weight * coordinateScale,
                                      control.y / control.weight * coordinateScale,
                                      control.z / control.weight * coordinateScale};
                if (!point.IsValid())
                {
                    return false;
                }
                bounds.ExpandToInclude(point);
            }
            return bounds.IsValid();
        }

        void SplitBezierControls(const std::vector<HomogeneousCurvePoint>& controls,
                                 std::vector<HomogeneousCurvePoint>& left,
                                 std::vector<HomogeneousCurvePoint>& right)
        {
            const std::size_t degree = controls.size() - 1;
            auto work = controls;
            left.resize(controls.size());
            right.resize(controls.size());
            for (std::size_t level = 0; level <= degree; ++level)
            {
                left[level] = work[0];
                right[degree - level] = work[degree - level];
                for (std::size_t j = 0; j < degree - level; ++j)
                {
                    work[j] = Interpolate(work[j], work[j + 1], 0.5);
                }
            }
        }

        SCSurfaceProjection3d RefineSurfaceProjection(
            const SCPoint3d& point, const ISCSurface& surface, double u, double v,
            const SCGeometryTolerance3d& tolerance,
            const SCIntervald* uCell = nullptr, const SCIntervald* vCell = nullptr,
            const SCNurbsSurface* validatedNurbs = nullptr,
            Detail::SurfaceSearchBudget* budget = nullptr,
            const SCPolygon2d* region = nullptr)
        {
            const SCIntervald uRange = uCell != nullptr ? *uCell :
                validatedNurbs != nullptr ? SCIntervald{validatedNurbs->KnotsU()[validatedNurbs->DegreeU()],
                    validatedNurbs->KnotsU()[validatedNurbs->ControlPointCountU()]} : surface.URange();
            const SCIntervald vRange = vCell != nullptr ? *vCell :
                validatedNurbs != nullptr ? SCIntervald{validatedNurbs->KnotsV()[validatedNurbs->DegreeV()],
                    validatedNurbs->KnotsV()[validatedNurbs->ControlPointCountV()]} : surface.VRange();
            const auto evaluate = [&](double candidateU, double candidateV, int order) {
                if (validatedNurbs != nullptr && budget != nullptr)
                    return budget->Evaluate(*validatedNurbs, candidateU, candidateV, order);
                if (budget != nullptr && !budget->Consume(order))
                {
                    SCSurfaceEval3d invalid{};
                    invalid.point.x = std::numeric_limits<double>::quiet_NaN();
                    return invalid;
                }
                return validatedNurbs != nullptr
                    ? Detail::EvaluateValidatedNurbsSurface(*validatedNurbs, candidateU, candidateV, order)
                    : surface.Evaluate(candidateU, candidateV, order);
            };
            const auto pointAt = [&](double candidateU, double candidateV) {
                if (region!=nullptr && LocatePoint(SCPoint2d{candidateU,candidateV},*region,tolerance.parameterEpsilon)==SCPointContainment2d::Outside)
                    return SCPoint3d{std::numeric_limits<double>::quiet_NaN(),0,0};
                return validatedNurbs != nullptr
                    ? budget!=nullptr ? budget->Position(*validatedNurbs,candidateU,candidateV) : evaluate(candidateU, candidateV, 0).point
                    : [&] {
                        if (budget != nullptr && !budget->Consume(0))
                            return SCPoint3d{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
                        return surface.PointAt(candidateU, candidateV);
                    }();
            };
            const auto distanceTo = [&](const SCPoint3d& candidate) {
                return candidate.IsValid()
                    ? std::hypot(point.x - candidate.x, point.y - candidate.y, point.z - candidate.z)
                    : std::numeric_limits<double>::infinity();
            };
            double bestU = std::clamp(u, uRange.min, uRange.max);
            double bestV = std::clamp(v, vRange.min, vRange.max);
            SCPoint3d bestPoint = pointAt(bestU, bestV);
            if (!bestPoint.IsValid())
            {
                return {};
            }
            double bestDistance = distanceTo(bestPoint);
            if (!std::isfinite(bestDistance))
            {
                return {};
            }
            bool usePointOnlyRefinement = dynamic_cast<const SCOffsetSurface*>(&surface) != nullptr;

            for (int iteration = 0; iteration < 32 && !usePointOnlyRefinement &&
                 (budget == nullptr || !budget->statistics.searchIncomplete); ++iteration)
            {
                const SCSurfaceEval3d eval = evaluate(bestU, bestV, 1);
                if (!Detail::HasIndependentSurfaceTangents(eval, tolerance.angleEpsilon) ||
                    !Detail::SameRoundedPosition(eval.point, bestPoint))
                {
                    usePointOnlyRefinement = true;
                    break;
                }

                const double scale = std::max({std::abs(eval.derivativeU.x), std::abs(eval.derivativeU.y),
                    std::abs(eval.derivativeU.z), std::abs(eval.derivativeV.x),
                    std::abs(eval.derivativeV.y), std::abs(eval.derivativeV.z)});
                const SCVector3d residual = (bestPoint - point) / scale;
                const SCVector3d du = eval.derivativeU / scale;
                const SCVector3d dv = eval.derivativeV / scale;
                const double uu = Dot(du, du);
                const double uv = Dot(du, dv);
                const double vv = Dot(dv, dv);
                const double uGradient = Dot(du, residual);
                const double vGradient = Dot(dv, residual);
                const double determinantScale = uu * vv;
                const double determinant = determinantScale - uv * uv;
                if (!std::isfinite(determinant) ||
                    determinant <= std::max(tolerance.angleEpsilon * tolerance.angleEpsilon, std::numeric_limits<double>::epsilon()) *
                                       determinantScale)
                {
                    usePointOnlyRefinement = true;
                    break;
                }

                const double deltaU = (-uGradient * vv + vGradient * uv) / determinant;
                const double deltaV = (-vGradient * uu + uGradient * uv) / determinant;
                if (!std::isfinite(deltaU) || !std::isfinite(deltaV))
                {
                    usePointOnlyRefinement = true;
                    break;
                }

                bool improved = false;
                for (double scale = 1.0; scale >= 1.0 / 128.0; scale *= 0.5)
                {
                    const double candidateU = std::clamp(bestU + scale * deltaU, uRange.min, uRange.max);
                    const double candidateV = std::clamp(bestV + scale * deltaV, vRange.min, vRange.max);
                    const SCPoint3d candidatePoint = pointAt(candidateU, candidateV);
                    const double candidateDistance = distanceTo(candidatePoint);
                    if (candidateDistance < bestDistance)
                    {
                        bestU = candidateU;
                        bestV = candidateV;
                        bestPoint = candidatePoint;
                        bestDistance = candidateDistance;
                        improved = true;
                        break;
                    }
                }
                if (!improved)
                {
                    usePointOnlyRefinement = true;
                    break;
                }
            }

            if (usePointOnlyRefinement)
            {
                double stepU = std::max((uRange.max*0.25-uRange.min*0.25), tolerance.parameterEpsilon);
                double stepV = std::max((vRange.max*0.25-vRange.min*0.25), tolerance.parameterEpsilon);
                for (int iteration = 0; iteration < 64 &&
                     (budget == nullptr || !budget->statistics.searchIncomplete); ++iteration)
                {
                    bool improved = false;
                    std::array<std::pair<double,double>,9> visited{};
                    std::size_t visitedCount = 0;
                    for (const double du : std::array<double, 3>{-stepU, 0.0, stepU})
                    {
                        for (const double dv : std::array<double, 3>{-stepV, 0.0, stepV})
                        {
                            const double candidateU = std::clamp(bestU + du, uRange.min, uRange.max);
                            const double candidateV = std::clamp(bestV + dv, vRange.min, vRange.max);
                            if (candidateU == bestU && candidateV == bestV) continue;
                            bool duplicate = false;
                            for (std::size_t i=0; i<visitedCount; ++i)
                                if (visited[i].first==candidateU && visited[i].second==candidateV) { duplicate=true; break; }
                            if (duplicate) continue;
                            visited[visitedCount++]={candidateU,candidateV};
                            const SCPoint3d candidatePoint = pointAt(candidateU, candidateV);
                            const double candidateDistance = distanceTo(candidatePoint);
                            if (candidateDistance < bestDistance)
                            {
                                bestU = candidateU;
                                bestV = candidateV;
                                bestPoint = candidatePoint;
                                bestDistance = candidateDistance;
                                improved = true;
                            }
                        }
                    }
                    if (!improved)
                    {
                        stepU *= 0.5;
                        stepV *= 0.5;
                    }
                    if (stepU <= tolerance.parameterEpsilon && stepV <= tolerance.parameterEpsilon)
                    {
                        break;
                    }
                }
            }

            const double bestDistanceSquared = bestDistance * bestDistance;
            return bestPoint.IsValid() && std::isfinite(bestDistance)
                ? SCSurfaceProjection3d{true, bestPoint, bestU, bestV, bestDistanceSquared}
                : SCSurfaceProjection3d{};
        }

        SCLineProjection3d ProjectPointToSegment3d(const SCPoint3d& point,
                                                   const SCPoint3d& start,
                                                   const SCPoint3d& end,
                                                   double eps)
        {
            const SCVector3d direction = end - start;
            const double length = std::hypot(direction.x,direction.y,direction.z);
            if (!std::isfinite(length)) return {{std::numeric_limits<double>::quiet_NaN(),0,0},0,0,false};
            if (length <= eps)
            {
                return {start, 0.0, (point - start).LengthSquared(), true};
            }

            const double raw=Dot(point-start,direction/length)/length;
            if (!std::isfinite(raw)) return {{std::numeric_limits<double>::quiet_NaN(),0,0},0,0,false};
            const double parameter = std::clamp(raw,0.0,1.0);
            const SCPoint3d projectedPoint{std::lerp(start.x,end.x,parameter),std::lerp(start.y,end.y,parameter),
                std::lerp(start.z,end.z,parameter)};
            return {projectedPoint, parameter, (point - projectedPoint).LengthSquared(), true};
        }

        bool UpdateClosestTrimProjection(const SCPoint3d& point,
                                         const SCCurveOnSurface& trim,
                                         SCBrepFaceProjection3d& best,
                                         bool onBoundary,
                                         const SCGeometryTolerance3d& tolerance,
                                         bool* analyticComplete = nullptr)
        {
            if (!point.IsValid() || !trim.IsValid(tolerance) || trim.PointCount() < 2)
            {
                if (analyticComplete != nullptr) *analyticComplete = false;
                return false;
            }

            bool improved = false;
            const std::size_t pointCount = trim.PointCount();
            const std::size_t segmentCount = trim.UvCurve().IsClosed() ? pointCount : pointCount - 1;
            for (std::size_t i = 0; i < segmentCount; ++i)
            {
                const Detail::CurveOnSurfaceProjectionCandidate projected =
                    Detail::ProjectPointToCurveOnSurfaceSegment(point, trim, i);
                if (analyticComplete != nullptr && !projected.analyticComplete)
                    *analyticComplete = false;
                if (projected.valid && (!best.success || projected.distance < CandidateDistance(point,best.point)))
                {
                    best.success = true;
                    best.onTrimmedFace = true;
                    best.onBoundary = onBoundary;
                    best.point = projected.sample.point;
                    best.u = projected.sample.uv.x;
                    best.v = projected.sample.uv.y;
                    best.distanceSquared = projected.distanceSquared;
                    improved = true;
                }
            }

            return improved;
        }

        SCBrepEdgeProjection3d RefineCurveProjection(const SCPoint3d& point,
                                                   const ISCCurve3d& curve,
                                                   double initialParameter,
                                                   const SCGeometryTolerance3d& tolerance,
                                                   bool pointOnlyFallback = true,
                                                   const SCNurbsCurve3d* validatedNurbs = nullptr)
        {
            const SCIntervald range = validatedNurbs != nullptr
                                          ? SCIntervald{validatedNurbs->Knots()[validatedNurbs->Degree()],
                                                        validatedNurbs->Knots()[validatedNurbs->ControlPoints().size()]}
                                          : curve.ParameterRange();
            if (!range.IsValid())
            {
                return {};
            }

            const auto evaluate = [&](double parameter, int derivativeOrder) {
                return validatedNurbs != nullptr
                           ? Detail::EvaluateValidatedNurbsCurve(*validatedNurbs, parameter, derivativeOrder)
                           : curve.Evaluate(parameter, derivativeOrder);
            };
            const auto pointAt = [&](double parameter) {
                return validatedNurbs != nullptr ? evaluate(parameter, 0).point : curve.PointAt(parameter);
            };

            double bestParameter = std::clamp(initialParameter, range.min, range.max);
            SCPoint3d bestPoint = pointAt(bestParameter);
            if (!bestPoint.IsValid())
            {
                return {};
            }
            double bestDistance = CandidateDistance(point,bestPoint);
            double bestDistanceSquared = bestDistance*bestDistance;
            if (!std::isfinite(bestDistance))
            {
                return {};
            }
            bool usePointOnlyRefinement = false;

            for (int iteration = 0; iteration < 32; ++iteration)
            {
                const SCCurveEval3d eval = evaluate(bestParameter, 2);
                if (!eval.IsValid() || eval.derivativeOrder < 1 || !Detail::SameRoundedPosition(eval.point, bestPoint))
                {
                    usePointOnlyRefinement = true;
                    break;
                }

                const SCVector3d residual = bestPoint - point;
                const double gradient = Dot(residual, eval.firstDerivative);
                const double tangentSquared = Dot(eval.firstDerivative, eval.firstDerivative);
                const double curvatureTerm = eval.derivativeOrder >= 2
                                                 ? Dot(residual, eval.secondDerivative)
                                                 : 0.0;
                const double hessian = tangentSquared + curvatureTerm;
                const double hessianScale = tangentSquared + std::abs(curvatureTerm);
                if (gradient == 0.0 && hessian > 0.0) break;
                const double denominator = hessian > std::numeric_limits<double>::epsilon() * hessianScale
                                               ? hessian
                                               : tangentSquared;
                if (!std::isfinite(denominator) || denominator <= 0.0)
                {
                    usePointOnlyRefinement = true;
                    break;
                }

                const double delta = -gradient / denominator;
                if (!std::isfinite(delta))
                {
                    usePointOnlyRefinement = true;
                    break;
                }

                bool improved = false;
                for (double scale = 1.0; scale >= 1.0 / 128.0; scale *= 0.5)
                {
                    const double candidateParameter =
                        std::clamp(bestParameter + scale * delta, range.min, range.max);
                    const SCPoint3d candidatePoint = pointAt(candidateParameter);
                    const double candidateDistance = CandidateDistance(point,candidatePoint);
                    const double candidateDistanceSquared = candidateDistance*candidateDistance;
                    if (candidatePoint.IsValid() && std::isfinite(candidateDistance) && candidateDistance < bestDistance)
                    {
                        bestParameter = candidateParameter;
                        bestPoint = candidatePoint;
                        bestDistanceSquared = candidateDistanceSquared;
                        bestDistance = candidateDistance;
                        improved = true;
                        break;
                    }
                }
                if (!improved)
                {
                    usePointOnlyRefinement = true;
                    break;
                }
            }

            if (usePointOnlyRefinement && pointOnlyFallback)
            {
                double step = std::max((range.max*0.25-range.min*0.25), tolerance.parameterEpsilon);
                for (int iteration = 0; iteration < 64; ++iteration)
                {
                    bool improved = false;
                    for (const double dt : std::array<double, 2>{-step, step})
                    {
                        const double candidateParameter = std::clamp(bestParameter + dt, range.min, range.max);
                        const SCPoint3d candidatePoint = pointAt(candidateParameter);
                        const double candidateDistance = CandidateDistance(point,candidatePoint);
                    const double candidateDistanceSquared = candidateDistance*candidateDistance;
                        if (candidatePoint.IsValid() && std::isfinite(candidateDistance) && candidateDistance < bestDistance)
                        {
                            bestParameter = candidateParameter;
                            bestPoint = candidatePoint;
                            bestDistanceSquared = candidateDistanceSquared;
                        bestDistance = candidateDistance;
                            improved = true;
                        }
                    }
                    if (!improved)
                    {
                        step *= 0.5;
                    }
                    if (step <= tolerance.parameterEpsilon)
                    {
                        break;
                    }
                }
            }

            bestPoint = pointAt(bestParameter);
            bestDistance = CandidateDistance(point,bestPoint);
            bestDistanceSquared = bestDistance*bestDistance;
            return bestPoint.IsValid() && std::isfinite(bestDistance)
                ? SCBrepEdgeProjection3d{true, bestParameter, bestPoint, bestDistanceSquared}
                : SCBrepEdgeProjection3d{};
        }

        SCPoint3d ClosestPointOnTriangle(const SCPoint3d& point, const SCTriangle3d& triangle, double /*eps*/)
        {
            const SCVector3d worldAb = triangle.b - triangle.a;
            const SCVector3d worldAc = triangle.c - triangle.a;
            const double scale=std::max({std::abs(worldAb.x),std::abs(worldAb.y),std::abs(worldAb.z),
                std::abs(worldAc.x),std::abs(worldAc.y),std::abs(worldAc.z)});
            if (!(scale>0) || !std::isfinite(scale)) return {std::numeric_limits<double>::quiet_NaN(),0,0};
            const SCVector3d ab=worldAb/scale, ac=worldAc/scale;
            const SCVector3d ap=(point-triangle.a)/scale;
            if (!ap.IsValid()) return {std::numeric_limits<double>::quiet_NaN(),0,0};

            const double d1 = Dot(ab, ap);
            const double d2 = Dot(ac, ap);
            if (d1 <= 0.0 && d2 <= 0.0)
            {
                return triangle.a;
            }

            const SCVector3d bp = (point - triangle.b)/scale;
            const double d3 = Dot(ab, bp);
            const double d4 = Dot(ac, bp);
            if (d3 >= 0.0 && d4 <= d3)
            {
                return triangle.b;
            }

            const double vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
            {
                const double v = d1 / (d1 - d3);
                return triangle.a + worldAb * v;
            }

            const SCVector3d cp = (point - triangle.c)/scale;
            const double d5 = Dot(ab, cp);
            const double d6 = Dot(ac, cp);
            if (d6 >= 0.0 && d5 <= d6)
            {
                return triangle.c;
            }

            const double vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
            {
                const double w = d2 / (d2 - d6);
                return triangle.a + worldAc * w;
            }

            const double va = d3 * d6 - d5 * d4;
            if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
            {
                const SCVector3d bc = triangle.c - triangle.b;
                const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                return triangle.b + bc * w;
            }

            const SCVector3d normal = Cross(ab, ac);
            const double normalLengthSquared = normal.LengthSquared();
            if (!(normalLengthSquared > 0.0))
            {
                return {std::numeric_limits<double>::quiet_NaN(),0,0};
            }

            const double distance = Dot(ap, normal) / normalLengthSquared;
            return point - normal * (distance*scale);
        }

    }  // namespace

    SCLineProjection3d ProjectPointToLine(const SCPoint3d& point, const SCLine3d& line, const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !point.IsValid() || !line.IsValid(tolerance.distanceEpsilon))
            return {line.origin,0,std::numeric_limits<double>::infinity(),false};
        const double scale=std::max({std::abs(line.direction.x),std::abs(line.direction.y),std::abs(line.direction.z)});
        const auto scaled=line.direction/scale;
        const double length=std::hypot(scaled.x,scaled.y,scaled.z);
        const auto unit=scaled/length;
        const double parameter=(Dot(point-line.origin,unit)/length)/scale;
        const auto projected=line.PointAt(parameter);
        const double distance=CandidateDistance(point,projected);
        if (!std::isfinite(parameter) || !projected.IsValid() || !std::isfinite(distance))
            return {line.origin,0,std::numeric_limits<double>::infinity(),false};
        return {projected,parameter,distance*distance,true};
    }

    SCCurveProjection3d Detail::SolveCurveProjection(const SCPoint3d& point,
                                          const ISCCurve3d& curve,
                                          const SCGeometryTolerance3d& tolerance)
    {
        if (!point.IsValid() || !curve.IsValid(tolerance))
        {
            return {};
        }

        if (const auto* lineCurve = dynamic_cast<const SCLineCurve3d*>(&curve))
        {
            const SCLineProjection3d lineProjection = ProjectPointToLine(point, lineCurve->Line(), tolerance);
            const SCIntervald range = lineCurve->ParameterRange();
            const double parameter = std::clamp(lineProjection.parameter, range.min, range.max);
            const SCPoint3d projectedPoint = lineCurve->PointAt(parameter);
            const double squared = (point - projectedPoint).LengthSquared();
            return projectedPoint.IsValid() && std::isfinite(CandidateDistance(point,projectedPoint))
                ? SCCurveProjection3d{true, projectedPoint, parameter, squared}
                : SCCurveProjection3d{};
        }

        const SCIntervald range = curve.ParameterRange();
        if (!range.IsValid())
        {
            return {};
        }

        std::vector<double> parameterBoundaries{range.min, range.max};
        const auto* nurbsCurve = dynamic_cast<const SCNurbsCurve3d*>(&curve);
        if (nurbsCurve != nullptr)
        {
            auto nurbsBoundaries = Detail::KnotSpanBoundaries(
                range, nurbsCurve->Knots(), nurbsCurve->Degree(), nurbsCurve->ControlPoints().size());
            if (nurbsBoundaries.size() >= 2)
            {
                parameterBoundaries = std::move(nurbsBoundaries);
            }
        }

        if (nurbsCurve != nullptr && parameterBoundaries.size() - 1 > 256 / 3)
        {
            struct SpanCandidate
            {
                double lowerBoundSquared;
                double start;
                double end;
            };
            std::vector<SpanCandidate> candidates;
            const auto& knots = nurbsCurve->Knots();
            const auto& controlPoints = nurbsCurve->ControlPoints();
            const std::size_t degree = static_cast<std::size_t>(nurbsCurve->Degree());
            candidates.reserve(16 * (controlPoints.size() - degree));
            const auto lowerBoundSquared = [&](const SCBox3d& box) {
                const SCPoint3d nearest{
                    std::clamp(point.x, box.MinPoint().x, box.MaxPoint().x),
                    std::clamp(point.y, box.MinPoint().y, box.MaxPoint().y),
                    std::clamp(point.z, box.MinPoint().z, box.MaxPoint().z)};
                const double squared = (point - nearest).LengthSquared();
                return std::isfinite(squared) ? squared : 0.0;
            };
            const auto subdivide = [&](const auto& self,
                                       const std::vector<HomogeneousCurvePoint>& controls,
                                       double start,
                                       double end,
                                       double coordinateScale,
                                       int depth,
                                       std::vector<SpanCandidate>& refined) -> bool {
                if (depth > 0)
                {
                    const double middle = std::lerp(start,end,0.5);
                    if (!(middle > start && middle < end))
                    {
                        return false;
                    }
                    std::vector<HomogeneousCurvePoint> left;
                    std::vector<HomogeneousCurvePoint> right;
                    SplitBezierControls(controls, left, right);
                    return self(self, left, start, middle, coordinateScale, depth - 1, refined) &&
                           self(self, right, middle, end, coordinateScale, depth - 1, refined);
                }
                SCBox3d bounds;
                if (!BoundHomogeneousControls(controls, coordinateScale, bounds))
                {
                    return false;
                }
                refined.push_back({lowerBoundSquared(bounds), start, end});
                return true;
            };
            for (std::size_t span = degree; span < controlPoints.size(); ++span)
            {
                if (knots[span + 1] <= knots[span])
                {
                    continue;
                }
                SCBox3d spanBounds;
                for (std::size_t index = span - degree; index <= span; ++index)
                {
                    spanBounds.ExpandToInclude(controlPoints[index]);
                }
                std::vector<HomogeneousCurvePoint> bezierControls;
                double coordinateScale = 1.0;
                std::vector<SpanCandidate> refined;
                if (degree <= 8 &&
                    BezierControlsForNurbsSpan(*nurbsCurve, span, bezierControls, coordinateScale) &&
                    subdivide(subdivide, bezierControls, knots[span], knots[span + 1], coordinateScale, 4, refined))
                {
                    candidates.insert(candidates.end(), refined.begin(), refined.end());
                }
                else
                {
                    candidates.push_back({lowerBoundSquared(spanBounds), knots[span], knots[span + 1]});
                }
            }
            std::sort(candidates.begin(), candidates.end(), [](const SpanCandidate& lhs, const SpanCandidate& rhs) {
                return lhs.lowerBoundSquared < rhs.lowerBoundSquared ||
                    (lhs.lowerBoundSquared==rhs.lowerBoundSquared && lhs.start<rhs.start);
            });

            SCBrepEdgeProjection3d best{};
            for (const SpanCandidate& candidate : candidates)
            {
                for (const double parameter : {candidate.start,
                                               std::lerp(candidate.start,candidate.end,0.5),
                                               candidate.end})
                {
                    const SCBrepEdgeProjection3d refined =
                        RefineCurveProjection(point, curve, parameter, tolerance, false, nurbsCurve);
                    if (refined.success && (!best.success || CandidateDistance(point,refined.point) < CandidateDistance(point,best.point)))
                    {
                        best = refined;
                    }
                }
            }
            if (best.success && CandidateDistance(point,best.point) > tolerance.distanceEpsilon)
            {
                const SCBrepEdgeProjection3d refined =
                    RefineCurveProjection(point, curve, best.parameter, tolerance, true, nurbsCurve);
                if (refined.success && CandidateDistance(point,refined.point) < CandidateDistance(point,best.point))
                {
                    best = refined;
                }
            }
            return {best.success, best.point, best.parameter, best.distanceSquared};
        }

        const SCBox3d curveBounds = curve.Bounds();
        const double curveScale = curveBounds.IsValid()
                                      ? (curveBounds.MaxPoint() - curveBounds.MinPoint()).Length()
                                      : tolerance.distanceEpsilon;
        const std::vector<double> parameterSeeds = Detail::AdaptiveCurveParameterSeeds(
            curve, std::move(parameterBoundaries), std::max(tolerance.distanceEpsilon * 4.0, curveScale * 1e-3));

        SCBrepEdgeProjection3d best{};
        for (const double parameter : parameterSeeds)
        {
            const SCBrepEdgeProjection3d refined =
                RefineCurveProjection(point, curve, parameter, tolerance, true, nurbsCurve);
            if (refined.success && (!best.success || CandidateDistance(point,refined.point) < CandidateDistance(point,best.point)))
            {
                best = refined;
            }
        }

        return {best.success, best.point, best.parameter, best.distanceSquared};
    }

    SCCurveOnSurfaceProjection3d Detail::SolveCurveOnSurfaceProjection(const SCPoint3d& point,
                                                            const SCCurveOnSurface& curveOnSurface,
                                                            const SCGeometryTolerance3d& tolerance,
                                                            bool* analyticComplete)
    {
        SCCurveOnSurfaceProjection3d best{};
        if (analyticComplete != nullptr) *analyticComplete = false;
        if (!point.IsValid() || !curveOnSurface.IsValid(tolerance) || curveOnSurface.PointCount() < 2)
        {
            return best;
        }

        const std::size_t pointCount = curveOnSurface.PointCount();
        const std::size_t segmentCount = curveOnSurface.UvCurve().IsClosed() ? pointCount : pointCount - 1;
        bool allAnalytic = dynamic_cast<const SCPlaneSurface*>(curveOnSurface.SupportSurface()) != nullptr;
        for (std::size_t i = 0; i < segmentCount; ++i)
        {
            const Detail::CurveOnSurfaceProjectionCandidate projected =
                Detail::ProjectPointToCurveOnSurfaceSegment(point, curveOnSurface, i);
            allAnalytic = allAnalytic && projected.analyticComplete;
            if (projected.valid && (!best.success || projected.distance < CandidateDistance(point,best.point)))
            {
                best.success = true;
                best.point = projected.sample.point;
                best.segmentIndex = i;
                best.segmentParameter = projected.sample.parameter;
                best.uv = projected.sample.uv;
                best.distanceSquared = projected.distanceSquared;
            }
        }

        if (analyticComplete != nullptr) *analyticComplete = allAnalytic && best.success;

        return best;
    }

    SCPlaneProjection3d ProjectPointToPlane(const SCPoint3d& point,
                                          const SCPlane& plane,
                                          const SCGeometryTolerance3d& tolerance)
    {
        if (!tolerance.IsValid() || !point.IsValid() || !plane.IsValid(tolerance.distanceEpsilon))
            return {plane.origin,0,std::numeric_limits<double>::infinity()};

        const SCVector3d unitNormal = plane.UnitNormal(tolerance.distanceEpsilon);
        const double signedDistance = Dot(point - plane.origin, unitNormal);
        const SCPoint3d projectedPoint = point - unitNormal * signedDistance;
        return SCPlaneProjection3d{projectedPoint, signedDistance, signedDistance * signedDistance};
    }

    SCSurfaceProjection3d Detail::SolveSurfaceProjection(const SCPoint3d& point,
                                              const ISCSurface& surface,
                                              const SCGeometryTolerance3d& tolerance, Detail::SurfaceSearchBudget& budget)
    {
        if (!tolerance.IsValid() || !point.IsValid() || !surface.IsValid(tolerance))
        {
            return {};
        }

        const SCIntervald uRange = surface.URange();
        const SCIntervald vRange = surface.VRange();
        if (!uRange.IsValid() || !vRange.IsValid())
        {
            return {};
        }

        if (const auto* planeSurface = dynamic_cast<const SCPlaneSurface*>(&surface))
        {
            const SCVector3d uAxis = planeSurface->UAxis();
            const SCVector3d vAxis = planeSurface->VAxis();
            SCVector3d unitU, unitV;
            if (!Detail::TryUnitVector(uAxis, unitU) || !Detail::TryUnitVector(vAxis, unitV))
            {
                return {};
            }
            const double uLength = std::hypot(uAxis.x, uAxis.y, uAxis.z);
            const double vLength = std::hypot(vAxis.x, vAxis.y, vAxis.z);
            if (!std::isfinite(uLength) || !std::isfinite(vLength))
            {
                return {};
            }
            SCSurfaceProjection3d best{};
            double bestDistance = std::numeric_limits<double>::infinity();
            const auto consider = [&](double u, double v) {
                const SCPoint3d candidate = planeSurface->PointAt(u, v);
                if (!candidate.IsValid())
                {
                    return;
                }
                const double distance = std::hypot(point.x - candidate.x, point.y - candidate.y,
                                                   point.z - candidate.z);
                const double squared = distance * distance;
                if (std::isfinite(distance) && (!best.success || distance < bestDistance))
                {
                    best = {true, candidate, u, v, squared};
                    bestDistance = distance;
                }
            };
            SCPoint2d stationary{};
            if (Detail::TryAffinePlaneUv(*planeSurface, point, tolerance, stationary) &&
                stationary.x >= uRange.min && stationary.x <= uRange.max &&
                stationary.y >= vRange.min && stationary.y <= vRange.max)
            {
                consider(stationary.x, stationary.y);
            }
            for (const double u : {uRange.min, uRange.max})
            {
                const SCPoint3d anchor = planeSurface->PointAt(u, 0.0);
                if (anchor.IsValid())
                {
                    const double v = Dot(point - anchor, unitV) / vLength;
                    if (std::isfinite(v))
                    {
                        consider(u, std::clamp(v, vRange.min, vRange.max));
                    }
                }
                consider(u, vRange.min);
                consider(u, vRange.max);
            }
            for (const double v : {vRange.min, vRange.max})
            {
                const SCPoint3d anchor = planeSurface->PointAt(0.0, v);
                if (anchor.IsValid())
                {
                    const double u = Dot(point - anchor, unitU) / uLength;
                    if (std::isfinite(u))
                    {
                        consider(std::clamp(u, uRange.min, uRange.max), v);
                    }
                }
            }
            return best;
        }

        std::vector<double> uBoundaries{uRange.min, uRange.max};
        std::vector<double> vBoundaries{vRange.min, vRange.max};
        const auto* nurbsSurface = dynamic_cast<const SCNurbsSurface*>(&surface);
        if (nurbsSurface != nullptr)
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
        if (nurbsSurface != nullptr && uBoundaries.size() - 1 > 256 / (vBoundaries.size() - 1))
        {
            budget.PreparePositionCache(*nurbsSurface);
            struct SurfaceCellCandidate
            {
                SCIntervald uCell;
                SCIntervald vCell;
            };
            std::vector<SurfaceCellCandidate> candidates;
            const auto& uKnots = nurbsSurface->KnotsU();
            const auto& vKnots = nurbsSurface->KnotsV();
            const std::size_t uCount = nurbsSurface->ControlPointCountU();
            const std::size_t vCount = nurbsSurface->ControlPointCountV();
            const std::size_t uDegree = static_cast<std::size_t>(nurbsSurface->DegreeU());
            const std::size_t vDegree = static_cast<std::size_t>(nurbsSurface->DegreeV());
            for (std::size_t vSpan = vDegree; vSpan < vCount; ++vSpan)
            {
                if (vKnots[vSpan + 1] <= vKnots[vSpan])
                {
                    continue;
                }
                for (std::size_t uSpan = uDegree; uSpan < uCount; ++uSpan)
                {
                    if (uKnots[uSpan + 1] <= uKnots[uSpan])
                    {
                        continue;
                    }
                    candidates.push_back({{uKnots[uSpan], uKnots[uSpan + 1]},
                                          {vKnots[vSpan], vKnots[vSpan + 1]}});
                }
            }

            SCSurfaceProjection3d best{};
            double bestDistance = std::numeric_limits<double>::infinity();
            std::size_t currentCell=0, bestCell=std::numeric_limits<std::size_t>::max();
            const auto preferredTie = [&](double u, double v) {
                return currentCell<bestCell || (currentCell==bestCell &&
                    (u<best.u || (u==best.u && v<best.v)));
            };
            const auto consider = [&](double u, double v) {
                const SCPoint3d candidatePoint = budget.Position(*nurbsSurface,u,v);
                if (!candidatePoint.IsValid())
                {
                    return;
                }
                const double distance = std::hypot(point.x - candidatePoint.x, point.y - candidatePoint.y,
                                                   point.z - candidatePoint.z);
                const double squared = distance * distance;
                if (std::isfinite(distance) && (!best.success || distance < bestDistance ||
                    (distance==bestDistance && preferredTie(u,v))))
                {
                    best = {true, candidatePoint, u, v, squared};
                    bestDistance = distance;
                    bestCell=currentCell;
                }
            };
            for (currentCell=0; currentCell<candidates.size(); ++currentCell)
            {
                const auto& candidate=candidates[currentCell];
                if (budget.statistics.searchIncomplete) break;
                const double uMiddle = std::lerp(candidate.uCell.min, candidate.uCell.max, 0.5);
                const double vMiddle = std::lerp(candidate.vCell.min, candidate.vCell.max, 0.5);
                const std::array<std::pair<double, double>, 5> seeds{{
                    {candidate.uCell.min, candidate.vCell.min},
                    {candidate.uCell.min, candidate.vCell.max},
                    {uMiddle, vMiddle},
                    {candidate.uCell.max, candidate.vCell.min},
                    {candidate.uCell.max, candidate.vCell.max}}};
                budget.statistics.seedCount += seeds.size();
                for (const auto& [u, v] : seeds)
                {
                    consider(u, v);
                    if (best.success && best.point == point)
                    {
                        return best;
                    }
                }
                for (const auto& [u, v] : seeds)
                {
                    const auto refined = RefineSurfaceProjection(point, surface, u, v, tolerance,
                                                                  &candidate.uCell, &candidate.vCell, nurbsSurface, &budget);
                    const double refinedDistance = refined.success
                        ? std::hypot(point.x - refined.point.x, point.y - refined.point.y,
                                     point.z - refined.point.z)
                        : std::numeric_limits<double>::infinity();
                    if (refined.success && refined.point.IsValid() &&
                        std::isfinite(refinedDistance) &&
                        (!best.success || refinedDistance < bestDistance ||
                         (refinedDistance==bestDistance && preferredTie(refined.u,refined.v))))
                    {
                        best = refined;
                        bestDistance = refinedDistance;
                        bestCell=currentCell;
                    }
                    if (best.success && best.point == point)
                    {
                        return best;
                    }
                }
            }
            return best;
        }
        const SCBox3d surfaceBounds = surface.Bounds();
        const double surfaceScale = surfaceBounds.IsValid()
                                        ? (surfaceBounds.MaxPoint() - surfaceBounds.MinPoint()).Length()
                                        : tolerance.distanceEpsilon;
        const std::vector<Detail::SurfaceParameterSeed> surfaceSeeds = Detail::AdaptiveSurfaceParameterSeeds(
            surface, std::move(uBoundaries), std::move(vBoundaries),
            std::max(tolerance.distanceEpsilon * 4.0, surfaceScale * 1e-2), 4, 256, &budget);

        budget.statistics.seedCount += surfaceSeeds.size();
        SCSurfaceProjection3d best{};
        double bestDistance = std::numeric_limits<double>::infinity();
        if (surfaceSeeds.empty())
        {
            return best;
        }

        const auto considerSeed = [&](const Detail::SurfaceParameterSeed& seed) {
            const SCSurfaceProjection3d refined = RefineSurfaceProjection(point, surface, seed.u, seed.v, tolerance, nullptr, nullptr, nurbsSurface, &budget);
            const double refinedDistance = refined.success
                ? std::hypot(point.x - refined.point.x, point.y - refined.point.y,
                             point.z - refined.point.z)
                : std::numeric_limits<double>::infinity();
            if (refined.success && refined.point.IsValid() &&
                std::isfinite(refinedDistance) &&
                (!best.success || refinedDistance < bestDistance))
            {
                best = refined;
                bestDistance = refinedDistance;
            }
        };

        std::vector<std::pair<double, std::size_t>> pointOnlySeeds;
        pointOnlySeeds.reserve(surfaceSeeds.size());
        for (std::size_t index = 0; index < surfaceSeeds.size(); ++index)
        {
            const auto& seed = surfaceSeeds[index];
            if (!budget.Consume(0)) break;
            const SCPoint3d seedPoint = nurbsSurface != nullptr
                ? Detail::EvaluateValidatedNurbsSurface(*nurbsSurface, seed.u, seed.v, 0).point
                : surface.PointAt(seed.u, seed.v);
            if (!seedPoint.IsValid())
            {
                continue;
            }
            const double distance = std::hypot(point.x - seedPoint.x, point.y - seedPoint.y,
                                               point.z - seedPoint.z);
            const double squared = distance * distance;
            if (!std::isfinite(distance))
            {
                continue;
            }
            if (std::isfinite(distance) && (!best.success || distance < bestDistance))
            {
                best = {true, seedPoint, seed.u, seed.v, squared};
                bestDistance = distance;
            }
            const bool isOffset = dynamic_cast<const SCOffsetSurface*>(&surface) != nullptr;
            if (!isOffset && !budget.Consume(1)) break;
            const SCSurfaceEval3d eval = isOffset ? SCSurfaceEval3d{} : nurbsSurface != nullptr
                ? Detail::EvaluateValidatedNurbsSurface(*nurbsSurface, seed.u, seed.v, 1)
                : surface.Evaluate(seed.u, seed.v, 1);
            if (isOffset || !Detail::HasIndependentSurfaceTangents(eval, tolerance.angleEpsilon) ||
                !Detail::SameRoundedPosition(eval.point, seedPoint))
            {
                pointOnlySeeds.emplace_back(distance, index);
            }
            else
            {
                considerSeed(seed);
            }
        }
        constexpr std::size_t kMaxPointOnlyRefinements = 16;
        const std::size_t refinementCount = std::min(kMaxPointOnlyRefinements, pointOnlySeeds.size());
        std::partial_sort(pointOnlySeeds.begin(), pointOnlySeeds.begin() + refinementCount, pointOnlySeeds.end());
        for (std::size_t i = 0; i < refinementCount; ++i)
        {
            considerSeed(surfaceSeeds[pointOnlySeeds[i].second]);
        }

        return best;
    }

    SCSurfaceProjection3d ProjectPointToSurface(const SCPoint3d& point, const ISCSurface& surface,
                                               const SCGeometryTolerance3d& tolerance)
    {
        Detail::SurfaceSearchBudget budget{};
        const auto candidate = Detail::SolveSurfaceProjection(point, surface, tolerance, budget);
        return candidate.success && candidate.IsValid() ? candidate : SCSurfaceProjection3d{};
    }

    SCSurfaceProjection3dWithStatus ProjectPointToSurfaceWithStatus(
        const SCPoint3d& point, const ISCSurface& surface, const SCGeometryTolerance3d& tolerance)
    {
        SCSurfaceProjection3dWithStatus result{};
        if (!tolerance.IsValid() || !point.IsValid() || !surface.IsValid(tolerance)) return result;
        Detail::SurfaceSearchBudget budget{};
        budget.bounded = dynamic_cast<const SCNurbsSurface*>(&surface) != nullptr;
        const auto candidate = Detail::SolveSurfaceProjection(point, surface, tolerance, budget);
        result.statistics = budget.statistics;
        result.state = SCGeometryOperationState::Unknown;
        result.evidence.optimality = SCGeometryEvidenceState::Unknown;
        result.evidence.reasons = SCGeometryFailureReason::NoProof;
        if (budget.statistics.searchIncomplete)
            result.evidence.reasons |= SCGeometryFailureReason::SearchIncomplete;
        if (!candidate.success) {
            result.evidence.reasons |= SCGeometryFailureReason::EvaluationUnavailable;
            return result;
        }
        if (!candidate.IsValid()) {
            result.evidence.reasons |= SCGeometryFailureReason::Unrepresentable;
            return result;
        }
        const auto actual = surface.PointAt(candidate.u, candidate.v);
        if (!Detail::SameRoundedPosition(actual, candidate.point)) {
            result.evidence.reasons |= SCGeometryFailureReason::EvaluationUnavailable;
            return result;
        }
        result.state = SCGeometryOperationState::HasResult;
        result.payloadValid = true;
        result.payload = candidate;
        result.evidence.hit = SCGeometryEvidenceState::Verified;
        if (dynamic_cast<const SCPlaneSurface*>(&surface) != nullptr) {
            result.evidence.optimality = SCGeometryEvidenceState::Verified;
            result.evidence.reasons = SCGeometryFailureReason::None;
        }
        return result;
    }

    SCBrepFaceProjection3d Detail::SolveFaceProjection(const SCPoint3d& point,
                                                const SCBrepFace& face,
                                                const SCGeometryTolerance3d& tolerance, bool bounded, SCGeometryEvidenceStatus* evidence,
                                                SCGeometrySearchStatistics* statistics)
    {
        SCBrepFaceProjection3d best{};
        if (!tolerance.IsValid() || !point.IsValid() || !face.IsValid(tolerance) || face.SupportSurface() == nullptr)
        {
            return best;
        }

        SCPolygon2d polygon{};
        if (!Detail::TryBuildPolygonFromBrepFaceTrims(face, tolerance, polygon))
        {
            return best;
        }

        const auto considerInterior = [&](double u, double v) {
            if (!std::isfinite(u) || !std::isfinite(v))
            {
                return;
            }
            const SCPointContainment2d containment =
                LocatePoint(SCPoint2d{u, v}, polygon, tolerance.parameterEpsilon);
            if (containment != SCPointContainment2d::Inside &&
                containment != SCPointContainment2d::OnBoundary)
            {
                return;
            }
            const SCPoint3d surfacePoint = face.SupportSurface()->PointAt(u, v);
            const double distance = std::hypot(point.x - surfacePoint.x,
                                               point.y - surfacePoint.y,
                                               point.z - surfacePoint.z);
            const double squared = distance * distance;
            if (surfacePoint.IsValid() && std::isfinite(distance) &&
                (!best.success || distance < CandidateDistance(point,best.point)))
            {
                best.success = true;
                best.onTrimmedFace = true;
                best.onBoundary = containment == SCPointContainment2d::OnBoundary;
                best.point = surfacePoint;
                best.u = u;
                best.v = v;
                best.distanceSquared = squared;
            }
        };
        const auto* planeSurface = dynamic_cast<const SCPlaneSurface*>(face.SupportSurface());
        bool planarAnalyticComplete = planeSurface != nullptr;
        if (planeSurface != nullptr)
        {
            SCPoint2d uv;
            if (Detail::TryAffinePlaneUv(*planeSurface, point, tolerance, uv))
            {
                considerInterior(uv.x, uv.y);
            }
            else planarAnalyticComplete = false;
        }
        else
        {
            Detail::SurfaceSearchBudget budget{};
            budget.bounded=bounded && dynamic_cast<const SCNurbsSurface*>(face.SupportSurface())!=nullptr;
            const SCSurfaceProjection3d surfaceProjection =
                Detail::SolveSurfaceProjection(point, *face.SupportSurface(), tolerance,budget);
            if (surfaceProjection.success)
            {
                considerInterior(surfaceProjection.u, surfaceProjection.v);
            }
            // 无约束候选在 trim 外时仍搜索区域内的驻点。
            if (!budget.statistics.searchIncomplete) {
                const auto& support=*face.SupportSurface();
                const auto* nurbs=dynamic_cast<const SCNurbsSurface*>(&support);
                std::vector<double> ub{support.URange().min,support.URange().max};
                std::vector<double> vb{support.VRange().min,support.VRange().max};
                if (nurbs!=nullptr) {
                    ub=Detail::KnotSpanBoundaries(support.URange(),nurbs->KnotsU(),nurbs->DegreeU(),nurbs->ControlPointCountU());
                    vb=Detail::KnotSpanBoundaries(support.VRange(),nurbs->KnotsV(),nurbs->DegreeV(),nurbs->ControlPointCountV());
                }
                auto seeds=Detail::AdaptiveSurfaceParameterSeeds(support,std::move(ub),std::move(vb),
                    tolerance.distanceEpsilon*4,4,256,&budget);
                const auto center=polygon.Centroid();
                const auto ringSeeds=[&](const SCPolyline2d& ring) {
                    for (std::size_t i=0;i<ring.PointCount();++i) {
                        const auto p=ring.PointAt(i);
                        seeds.push_back({std::lerp(p.x,center.x,0.01),std::lerp(p.y,center.y,0.01)});
                    }
                };
                ringSeeds(polygon.OuterRing());
                for (std::size_t i=0;i<polygon.HoleCount();++i) ringSeeds(polygon.HoleAt(i));
                budget.statistics.seedCount += seeds.size();
                for (const auto& seed:seeds) {
                    if (budget.statistics.searchIncomplete) break;
                    if (LocatePoint(SCPoint2d{seed.u,seed.v},polygon,tolerance.parameterEpsilon)==SCPointContainment2d::Outside) continue;
                    const auto local=RefineSurfaceProjection(point,support,seed.u,seed.v,tolerance,nullptr,nullptr,nurbs,&budget,&polygon);
                    if (local.success) considerInterior(local.u,local.v);
                }
            }
            if (budget.statistics.searchIncomplete && evidence!=nullptr) {
                evidence->optimality=SCGeometryEvidenceState::Unknown;
                evidence->reasons|=SCGeometryFailureReason::SearchIncomplete;
            }
            if (statistics!=nullptr) *statistics=budget.statistics;
        }

        UpdateClosestTrimProjection(point, face.OuterTrim(), best, true, tolerance,
                                    planeSurface != nullptr ? &planarAnalyticComplete : nullptr);
        for (const SCCurveOnSurface& holeTrim : face.HoleTrims())
        {
            UpdateClosestTrimProjection(point, holeTrim, best, true, tolerance,
                                        planeSurface != nullptr ? &planarAnalyticComplete : nullptr);
        }
        if (evidence != nullptr && planeSurface != nullptr) {
            evidence->optimality = planarAnalyticComplete && best.success
                ? SCGeometryEvidenceState::Verified : SCGeometryEvidenceState::Unknown;
            if (evidence->optimality == SCGeometryEvidenceState::Unknown)
                evidence->reasons |= SCGeometryFailureReason::NoProof;
        }
        return best;
    }

    SCBrepEdgeProjection3d Detail::SolveEdgeProjection(const SCPoint3d& point,
                                                const SCBrepEdge& edge,
                                                const SCGeometryTolerance3d& tolerance)
    {
        if (!edge.IsValid(tolerance) || edge.Curve() == nullptr)
        {
            return {};
        }

        const SCCurveProjection3d curveProjection = Detail::SolveCurveProjection(point, *edge.Curve(), tolerance);
        return {curveProjection.success,
                curveProjection.parameter,
                curveProjection.point,
                curveProjection.distanceSquared};
    }

    SCBrepVertexProjection3d ProjectPointToBrepVertex(const SCPoint3d& point,
                                                    const SCBrepVertex& vertex,
                                                    const SCGeometryTolerance3d& tolerance)
    {
        const SCBrepVertexProjection3dWithStatus result =
            ProjectPointToBrepVertexWithStatus(point, vertex, tolerance);
        return result.payloadValid ? result.payload : SCBrepVertexProjection3d{};
    }

    SCBrepVertexProjection3dWithStatus ProjectPointToBrepVertexWithStatus(
        const SCPoint3d& point, const SCBrepVertex& vertex, const SCGeometryTolerance3d& tolerance)
    {
        SCBrepVertexProjection3dWithStatus result{};
        if (!tolerance.IsValid() || !point.IsValid() || !vertex.IsValid())
        {
            return result;
        }

        const SCPoint3d vertexPoint = vertex.Point();
        const double distance = std::hypot(point.x - vertexPoint.x,
                                           point.y - vertexPoint.y,
                                           point.z - vertexPoint.z);
        const double squared = distance * distance;
        if (!std::isfinite(squared))
        {
            result.state = SCGeometryOperationState::Unknown;
            result.evidence.reasons = SCGeometryFailureReason::Unrepresentable;
            return result;
        }
        result.state = SCGeometryOperationState::HasResult;
        result.payloadValid = true;
        result.payload = {true, vertexPoint, squared};
        result.evidence.hit = SCGeometryEvidenceState::Verified;
        result.evidence.optimality = SCGeometryEvidenceState::Verified;
        result.evidence.reasons = SCGeometryFailureReason::None;
        return result;
    }

    SCBrepBodyProjection3d ProjectPointToBrepBody(const SCPoint3d& point, const SCBrepBody& body,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto result=Detail::SolveBodyProjection(point,body,tolerance,false);
        return result.validInput && result.allFacesAvailable && result.candidate.success && result.candidate.IsValid()
            ? result.candidate : SCBrepBodyProjection3d{};
    }

    SCPolyhedronFaceProjection3d Detail::SolvePolyFaceProjection(const SCPoint3d& point,
                                                            const PolyhedronFace3d& face,
                                                            const SCGeometryTolerance3d& tolerance)
    {
        SCPolyhedronFaceProjection3d best{};
        if (!tolerance.IsValid() || !point.IsValid() || !face.IsValid(tolerance.distanceEpsilon))
        {
            return best;
        }

        const SCFaceProjection3d faceProjection = ProjectFaceToPolygon2d(face, tolerance);
        if (!faceProjection.success)
        {
            return best;
        }

        const SCPlaneProjection3d planeProjection = ProjectPointToPlane(point, face.SupportPlane(), tolerance);
        const SCPoint2d uv = ProjectToLocalPlaneCoordinates(
            planeProjection.point, face.SupportPlane(), {faceProjection.uAxis, faceProjection.vAxis});
        const SCPointContainment2d containment = LocatePoint(uv, faceProjection.polygon, tolerance.distanceEpsilon);
        if (uv.IsValid() && planeProjection.point.IsValid() &&
            std::isfinite(CandidateDistance(point,planeProjection.point)) &&
            (containment == SCPointContainment2d::Inside || containment == SCPointContainment2d::OnBoundary))
        {
            best.success = true;
            best.onFace = true;
            best.onBoundary = containment == SCPointContainment2d::OnBoundary;
            best.point = planeProjection.point;
            best.u = uv.x;
            best.v = uv.y;
            best.distanceSquared = planeProjection.distanceSquared;
            return best;
        }

        auto updateLoop = [&](const PolyhedronLoop3d& loop) {
            const auto& vertices = loop.Vertices();
            for (std::size_t i = 0; i < vertices.size(); ++i)
            {
                const SCPoint3d& start = vertices[i];
                const SCPoint3d& end = vertices[(i + 1) % vertices.size()];
                const SCLineProjection3d projected =
                    ProjectPointToSegment3d(point, start, end, tolerance.distanceEpsilon);
                if (projected.point.IsValid() && std::isfinite(CandidateDistance(point,projected.point)) &&
                    (!best.success || CandidateDistance(point,projected.point) < CandidateDistance(point,best.point)))
                {
                    best.success = true;
                    best.onFace = false;
                    best.onBoundary = true;
                    best.point = projected.point;
                    best.distanceSquared = projected.distanceSquared;
                    const SCPoint2d uv0 = ProjectToLocalPlaneCoordinates(
                        start, face.SupportPlane(), {faceProjection.uAxis, faceProjection.vAxis});
                    const SCPoint2d uv1 = ProjectToLocalPlaneCoordinates(
                        end, face.SupportPlane(), {faceProjection.uAxis, faceProjection.vAxis});
                    const SCPoint2d edgeUv = uv0 + (uv1 - uv0) * projected.parameter;
                    best.u = edgeUv.x;
                    best.v = edgeUv.y;
                }
            }
        };

        updateLoop(face.OuterLoopRef());
        for (std::size_t i = 0; i < face.HoleCount(); ++i)
        {
            updateLoop(face.HoleAtRef(i));
        }
        return best;
    }

    SCPolyhedronBodyProjection3d ProjectPointToPolyhedronBody(const SCPoint3d& point, const PolyhedronBody& body,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto result=Detail::SolveBodyProjection(point,body,tolerance,false);
        return result.validInput && result.allFacesAvailable && result.candidate.success && result.candidate.IsValid()
            ? result.candidate : SCPolyhedronBodyProjection3d{};
    }

    SCTriangleMeshProjection3d Detail::SolveMeshProjection(const SCPoint3d& point,
                                                        const TriangleMesh& mesh,
                                                        const SCGeometryTolerance3d& tolerance)
    {
        SCTriangleMeshProjection3d best{};
        if (!tolerance.IsValid() || !point.IsValid() || !mesh.IsValid(tolerance.distanceEpsilon))
        {
            return best;
        }

        for (std::size_t triangleIndex = 0; triangleIndex < mesh.TriangleCount(); ++triangleIndex)
        {
            const SCTriangle3d triangle = mesh.TriangleAt(triangleIndex);
            const SCPoint3d projectedPoint = ClosestPointOnTriangle(point, triangle, tolerance.distanceEpsilon);
            const double distanceSquared = (point - projectedPoint).LengthSquared();
            if (projectedPoint.IsValid() && std::isfinite(CandidateDistance(point,projectedPoint)) &&
                (!best.success || std::hypot(point.x-projectedPoint.x, point.y-projectedPoint.y,
                 point.z-projectedPoint.z) < std::hypot(point.x-best.point.x, point.y-best.point.y, point.z-best.point.z)))
            {
                best.success = true;
                best.triangleIndex = triangleIndex;
                best.point = projectedPoint;
                best.distanceSquared = distanceSquared;
            }
        }

        return best;
    }

    Detail::BrepBodyCandidate Detail::SolveBodyProjection(const SCPoint3d& point, const SCBrepBody& body,
        const SCGeometryTolerance3d& tolerance, bool bounded)
    {
        BrepBodyCandidate result{};
        if (!tolerance.IsValid() || !point.IsValid() || !body.IsValid(tolerance)) return result;
        result.validInput=true;
        result.evidence.region=SCGeometryEvidenceState::Verified;
        result.evidence.optimality=SCGeometryEvidenceState::Verified;
        result.evidence.reasons=SCGeometryFailureReason::None;
        double bestDistance=std::numeric_limits<double>::infinity();
        std::size_t flat=0;
        for (std::size_t shell=0;shell<body.ShellCount();++shell) {
            const auto& current=body.ShellAtRef(shell);
            if (current.FaceCount()>std::numeric_limits<std::size_t>::max()-flat) {
                result.allFacesAvailable=false;
                result.evidence.region=SCGeometryEvidenceState::Unknown;
                result.evidence.optimality=SCGeometryEvidenceState::Unknown;
                result.evidence.reasons|=SCGeometryFailureReason::Unrepresentable;
                return result;
            }
            for (std::size_t local=0;local<current.FaceCount();++local,++flat) {
                SCGeometryEvidenceStatus evidence{};
                evidence.reasons=SCGeometryFailureReason::None;
                evidence.region=SCGeometryEvidenceState::Verified;
                SCBrepFace prepared;
                SCBrepFaceProjection3d candidate{};
                SCGeometrySearchStatistics statistics{};
                if (TryPrepareFaceInBody(body,current.FaceAtRef(local),tolerance,prepared)) {
                    const bool analytic=dynamic_cast<const SCPlaneSurface*>(prepared.SupportSurface())!=nullptr;
                    evidence.optimality=analytic ? SCGeometryEvidenceState::Verified : SCGeometryEvidenceState::Unknown;
                    if (!analytic) evidence.reasons|=SCGeometryFailureReason::NoProof;
                    candidate=SolveFaceProjection(point,prepared,tolerance,bounded,&evidence,&statistics);
                    if (candidate.success) evidence.hit=SCGeometryEvidenceState::Verified;
                    else {
                        evidence.optimality=SCGeometryEvidenceState::Unknown;
                        evidence.reasons|=SCGeometryFailureReason::EvaluationUnavailable;
                        result.allFacesAvailable=false;
                    }
                } else {
                    result.allFacesAvailable=false;
                    evidence.region=SCGeometryEvidenceState::Unknown;
                    evidence.optimality=SCGeometryEvidenceState::Unknown;
                    evidence.reasons=SCGeometryFailureReason::RegionUnavailable;
                }
                result.faces.push_back({shell,local,flat,candidate,evidence,statistics});
                result.evidence.reasons|=evidence.reasons;
                if (evidence.region!=SCGeometryEvidenceState::Verified) result.evidence.region=SCGeometryEvidenceState::Unknown;
                if (evidence.optimality!=SCGeometryEvidenceState::Verified) result.evidence.optimality=SCGeometryEvidenceState::Unknown;
                if (candidate.success && CandidateDistance(point,candidate.point)<bestDistance) {
                    bestDistance=CandidateDistance(point,candidate.point);
                    result.candidate={true,flat,candidate};
                    result.evidence.hit=SCGeometryEvidenceState::Verified;
                }
            }
        }
        return result;
    }

    Detail::PolyhedronBodyCandidate Detail::SolveBodyProjection(const SCPoint3d& point, const PolyhedronBody& body,
        const SCGeometryTolerance3d& tolerance, bool)
    {
        PolyhedronBodyCandidate result{};
        if (!tolerance.IsValid() || !point.IsValid() || !body.IsValid(tolerance)) return result;
        result.validInput=true;
        result.evidence.region=SCGeometryEvidenceState::Verified;
        result.evidence.optimality=SCGeometryEvidenceState::Verified;
        result.evidence.reasons=SCGeometryFailureReason::None;
        double bestDistance=std::numeric_limits<double>::infinity();
        for (std::size_t index=0;index<body.FaceCount();++index) {
            const auto candidate=SolvePolyFaceProjection(point,body.FaceAtRef(index),tolerance);
            result.faces.push_back(candidate);
            if (!candidate.success) {
                result.allFacesAvailable=false;
                result.evidence.optimality=SCGeometryEvidenceState::Unknown;
                result.evidence.reasons|=SCGeometryFailureReason::EvaluationUnavailable;
            } else if (CandidateDistance(point,candidate.point)<bestDistance) {
                bestDistance=CandidateDistance(point,candidate.point);
                result.candidate={true,index,candidate};
                result.evidence.hit=SCGeometryEvidenceState::Verified;
            }
        }
        return result;
    }

    SCBrepBodyProjection3dWithStatus ProjectPointToBrepBodyWithStatus(
        const SCPoint3d& point, const SCBrepBody& target, const SCGeometryTolerance3d& tolerance)
    {
        const auto internal=Detail::SolveBodyProjection(point,target,tolerance,true);
        SCBrepBodyProjection3dWithStatus result{};
        if (!internal.validInput) return result;
        result.state=SCGeometryOperationState::Unknown;
        result.evidence=internal.evidence;
        for (const auto& record:internal.faces) {
            SCBrepFaceProjection3dWithStatus face{};
            face.state=SCGeometryOperationState::Unknown;
            face.evidence=record.evidence;
            face.statistics=record.statistics;
            if (record.candidate.success && record.candidate.IsValid()) {
                face.state=SCGeometryOperationState::HasResult;
                face.payloadValid=true;
                face.payload=record.candidate;
            } else if (record.candidate.success) face.evidence.reasons|=SCGeometryFailureReason::Unrepresentable;
            result.faces.push_back({record.shellIndex,record.localFaceIndex,record.faceIndex,face});
        }
        if (internal.candidate.success && internal.candidate.projection.success && internal.candidate.IsValid()) {
            result.state=SCGeometryOperationState::HasResult;
            result.payloadValid=true;
            result.payload=internal.candidate;
        } else if (internal.candidate.success) result.evidence.reasons|=SCGeometryFailureReason::Unrepresentable;
        return result;
    }

    SCPolyhedronBodyProjection3dWithStatus ProjectPointToPolyhedronBodyWithStatus(
        const SCPoint3d& point, const PolyhedronBody& target, const SCGeometryTolerance3d& tolerance)
    {
        const auto internal=Detail::SolveBodyProjection(point,target,tolerance,true);
        SCPolyhedronBodyProjection3dWithStatus result{};
        if (!internal.validInput) return result;
        result.state=SCGeometryOperationState::Unknown;
        result.evidence=internal.evidence;
        for (const auto& candidate:internal.faces) {
            SCPolyhedronFaceProjection3dWithStatus face{};
            face.state=SCGeometryOperationState::Unknown;
            face.evidence.region=SCGeometryEvidenceState::Verified;
            face.evidence.optimality=SCGeometryEvidenceState::Unknown;
            face.evidence.reasons=SCGeometryFailureReason::EvaluationUnavailable;
            if (candidate.success) {
                face.evidence.hit=SCGeometryEvidenceState::Verified;
                face.evidence.optimality=SCGeometryEvidenceState::Verified;
                face.evidence.reasons=SCGeometryFailureReason::Unrepresentable;
                if (candidate.IsValid()) {
                    face.state=SCGeometryOperationState::HasResult;
                    face.payloadValid=true; face.payload=candidate;
                    face.evidence.reasons=SCGeometryFailureReason::None;
                }
            }
            result.faces.push_back(face);
        }
        if (internal.candidate.success && internal.candidate.projection.success && internal.candidate.IsValid()) {
            result.state=SCGeometryOperationState::HasResult;
            result.payloadValid=true;
            result.payload=internal.candidate;
        } else if (internal.candidate.success) result.evidence.reasons|=SCGeometryFailureReason::Unrepresentable;
        return result;
    }

    SCCurveProjection3d ProjectPointToCurve(const SCPoint3d& point, const ISCCurve3d& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto candidate=Detail::SolveCurveProjection(point,target,tolerance);
        return candidate.success && candidate.IsValid() ? candidate : SCCurveProjection3d{};
    }

    SCCurveOnSurfaceProjection3d ProjectPointToCurveOnSurface(const SCPoint3d& point, const SCCurveOnSurface& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto candidate=Detail::SolveCurveOnSurfaceProjection(point,target,tolerance);
        return candidate.success && candidate.IsValid() ? candidate : SCCurveOnSurfaceProjection3d{};
    }

    SCBrepFaceProjection3d ProjectPointToBrepFace(const SCPoint3d& point, const SCBrepFace& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto candidate=Detail::SolveFaceProjection(point,target,tolerance);
        return candidate.success && candidate.IsValid() ? candidate : SCBrepFaceProjection3d{};
    }

    SCBrepEdgeProjection3d ProjectPointToBrepEdge(const SCPoint3d& point, const SCBrepEdge& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto candidate=Detail::SolveEdgeProjection(point,target,tolerance);
        return candidate.success && candidate.IsValid() ? candidate : SCBrepEdgeProjection3d{};
    }

    SCPolyhedronFaceProjection3d ProjectPointToPolyhedronFace(const SCPoint3d& point, const PolyhedronFace3d& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto candidate=Detail::SolvePolyFaceProjection(point,target,tolerance);
        return candidate.success && candidate.IsValid() ? candidate : SCPolyhedronFaceProjection3d{};
    }

    SCTriangleMeshProjection3d ProjectPointToTriangleMesh(const SCPoint3d& point, const TriangleMesh& target,
        const SCGeometryTolerance3d& tolerance)
    {
        const auto candidate=Detail::SolveMeshProjection(point,target,tolerance);
        return candidate.success && candidate.IsValid() ? candidate : SCTriangleMeshProjection3d{};
    }

    namespace
    {
        template <typename Result, typename Payload>
        Result VerifiedProjectionCandidate(
            const SCPoint3d& query, Payload candidate, bool validInput, bool analytic,
            const SCPoint3d& actual, bool region = false)
        {
            Result result{};
            if (!validInput) return result;
            result.state = SCGeometryOperationState::Unknown;
            result.evidence.optimality = SCGeometryEvidenceState::Unknown;
            result.evidence.region = region ? SCGeometryEvidenceState::Verified : SCGeometryEvidenceState::NotApplicable;
            result.evidence.reasons = SCGeometryFailureReason::EvaluationUnavailable;
            if (!candidate.success || !actual.IsValid() ||
                !Detail::SameRoundedPosition(actual, candidate.point)) return result;
            const double distance = std::hypot(query.x - actual.x, query.y - actual.y, query.z - actual.z);
            const double squared = distance * distance;
            if (!std::isfinite(squared)) {
                result.evidence.reasons = SCGeometryFailureReason::Unrepresentable;
                return result;
            }
            candidate.point = actual;
            candidate.distanceSquared = squared;
            if (!candidate.IsValid()) return result;
            result.state = SCGeometryOperationState::HasResult;
            result.payloadValid = true;
            result.payload = candidate;
            result.evidence.hit = SCGeometryEvidenceState::Verified;
            result.evidence.optimality = analytic ? SCGeometryEvidenceState::Verified : SCGeometryEvidenceState::Unknown;
            result.evidence.reasons = analytic ? SCGeometryFailureReason::None : SCGeometryFailureReason::NoProof;
            return result;
        }
    }

    SCCurveProjection3dWithStatus ProjectPointToCurveWithStatus(
        const SCPoint3d& point, const ISCCurve3d& target, const SCGeometryTolerance3d& tolerance)
    {
        const bool valid = tolerance.IsValid() && point.IsValid() && target.IsValid(tolerance);
        const auto candidate = valid ? Detail::SolveCurveProjection(point, target, tolerance) : SCCurveProjection3d{};
        return VerifiedProjectionCandidate<SCCurveProjection3dWithStatus>(point, candidate, valid,
            dynamic_cast<const SCLineCurve3d*>(&target) != nullptr,
            candidate.success ? target.PointAt(candidate.parameter) : SCPoint3d{});
    }

    SCCurveOnSurfaceProjection3dWithStatus ProjectPointToCurveOnSurfaceWithStatus(
        const SCPoint3d& point, const SCCurveOnSurface& target, const SCGeometryTolerance3d& tolerance)
    {
        const bool valid = tolerance.IsValid() && point.IsValid() && target.IsValid(tolerance);
        bool analyticComplete = false;
        const auto candidate = valid ? Detail::SolveCurveOnSurfaceProjection(point, target, tolerance, &analyticComplete) : SCCurveOnSurfaceProjection3d{};
        return VerifiedProjectionCandidate<SCCurveOnSurfaceProjection3dWithStatus>(point, candidate, valid, analyticComplete,
            candidate.success ? target.SupportSurface()->PointAt(candidate.uv.x, candidate.uv.y) : SCPoint3d{});
    }

    SCBrepEdgeProjection3dWithStatus ProjectPointToBrepEdgeWithStatus(
        const SCPoint3d& point, const SCBrepEdge& target, const SCGeometryTolerance3d& tolerance)
    {
        const bool valid = tolerance.IsValid() && point.IsValid() && target.IsValid(tolerance);
        const auto candidate = valid ? Detail::SolveEdgeProjection(point, target, tolerance) : SCBrepEdgeProjection3d{};
        return VerifiedProjectionCandidate<SCBrepEdgeProjection3dWithStatus>(point, candidate, valid,
            dynamic_cast<const SCLineCurve3d*>(target.Curve()) != nullptr,
            candidate.success ? target.Curve()->PointAt(candidate.parameter) : SCPoint3d{});
    }

    SCBrepFaceProjection3dWithStatus ProjectPointToBrepFaceWithStatus(
        const SCPoint3d& point, const SCBrepFace& target, const SCGeometryTolerance3d& tolerance)
    {
        const bool valid = tolerance.IsValid() && point.IsValid() && target.IsValid(tolerance);
        if (!valid) return {};
        SCPolygon2d region;
        if (!Detail::TryBuildPolygonFromBrepFaceTrims(target, tolerance, region)) {
            SCBrepFaceProjection3dWithStatus result{};
            result.state = SCGeometryOperationState::Unknown;
            result.evidence.region = SCGeometryEvidenceState::Unknown;
            result.evidence.optimality = SCGeometryEvidenceState::Unknown;
            result.evidence.reasons = SCGeometryFailureReason::RegionUnavailable;
            return result;
        }
        SCGeometryEvidenceStatus searchEvidence{};
        searchEvidence.reasons=SCGeometryFailureReason::None;
        SCGeometrySearchStatistics statistics{};
        const auto candidate = Detail::SolveFaceProjection(point,target,tolerance,true,&searchEvidence,&statistics);
        auto result = VerifiedProjectionCandidate<SCBrepFaceProjection3dWithStatus>(point, candidate, true,
            dynamic_cast<const SCPlaneSurface*>(target.SupportSurface()) != nullptr,
            candidate.success ? target.SupportSurface()->PointAt(candidate.u, candidate.v) : SCPoint3d{}, true);
        result.evidence.reasons |= searchEvidence.reasons;
        result.statistics=statistics;
        if (searchEvidence.optimality==SCGeometryEvidenceState::Unknown) result.evidence.optimality=SCGeometryEvidenceState::Unknown;
        return result;
    }

    SCPolyhedronFaceProjection3dWithStatus ProjectPointToPolyhedronFaceWithStatus(
        const SCPoint3d& point, const PolyhedronFace3d& target, const SCGeometryTolerance3d& tolerance)
    {
        const bool valid = tolerance.IsValid() && point.IsValid() && target.IsValid(tolerance);
        const auto candidate = valid ? Detail::SolvePolyFaceProjection(point, target, tolerance) : SCPolyhedronFaceProjection3d{};
        const auto basis = valid ? ProjectFaceToPolygon2d(target, tolerance) : SCFaceProjection3d{};
        return VerifiedProjectionCandidate<SCPolyhedronFaceProjection3dWithStatus>(point, candidate, valid, true,
            basis.success ? basis.origin + basis.uAxis * candidate.u + basis.vAxis * candidate.v : SCPoint3d{}, true);
    }

    SCTriangleMeshProjection3dWithStatus ProjectPointToTriangleMeshWithStatus(
        const SCPoint3d& point, const TriangleMesh& target, const SCGeometryTolerance3d& tolerance)
    {
        const bool valid = tolerance.IsValid() && point.IsValid() && target.IsValid(tolerance.distanceEpsilon);
        const auto candidate = valid ? Detail::SolveMeshProjection(point, target, tolerance) : SCTriangleMeshProjection3d{};
        const auto actual = candidate.success && candidate.triangleIndex < target.TriangleCount()
            ? ClosestPointOnTriangle(point, target.TriangleAt(candidate.triangleIndex), tolerance.distanceEpsilon)
            : SCPoint3d{};
        return VerifiedProjectionCandidate<SCTriangleMeshProjection3dWithStatus>(point, candidate, valid, true, actual, true);
    }

    SCFaceProjection3d ProjectFaceToPolygon2d(const PolyhedronFace3d& face, const SCGeometryTolerance3d& tolerance)
    {
        if (!face.IsValid(tolerance.distanceEpsilon))
        {
            return {};
        }

        const SCPlane plane = face.SupportPlane();
        const PlaneProjectionBasis basis = BuildPlaneProjectionBasis(plane, tolerance.distanceEpsilon);
        const PolyhedronLoop3d& outerLoop = face.OuterLoopRef();
        std::vector<SCPoint2d> outerPoints;
        outerPoints.reserve(outerLoop.VertexCount());
        for (const SCPoint3d& vertex : outerLoop.Vertices())
        {
            outerPoints.push_back(ProjectToLocalPlaneCoordinates(vertex, plane, basis));
        }
        EnsureRingOrientation(outerPoints, true);

        std::vector<SCPolyline2d> holeRings;
        holeRings.reserve(face.HoleCount());
        for (std::size_t i = 0; i < face.HoleCount(); ++i)
        {
            std::vector<SCPoint2d> holePoints;
            const PolyhedronLoop3d& hole = face.HoleAtRef(i);
            holePoints.reserve(hole.VertexCount());
            for (const SCPoint3d& vertex : hole.Vertices())
            {
                holePoints.push_back(ProjectToLocalPlaneCoordinates(vertex, plane, basis));
            }
            EnsureRingOrientation(holePoints, false);
            holeRings.push_back(Detail::MakeVertexPreservingRing(holePoints));
        }

        SCPolygon2d polygon(Detail::MakeVertexPreservingRing(outerPoints), std::move(holeRings));
        if (!Triangulate(polygon, tolerance.distanceEpsilon))
        {
            return {};
        }

        return SCFaceProjection3d{true, std::move(polygon), plane.origin, basis.u, basis.v};
    }
}  // namespace Geometry
